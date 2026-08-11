# model_sparse4d.py  (single-view version; GroundHead **2D** with Cross-Attn & grid_sample-based height)
import math
import os
from typing import Optional, Tuple, Dict, Iterable

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from scipy.optimize import linear_sum_assignment
from ultralytics.nn.modules import Conv

# 你的工程内依赖（保持不变）
from .backbones.yolo11_backbone import YOLO11Backbone


class FastBatchFirstMHA(nn.Module):
    """
    精简版 Multi-Head Attention（batch_first 专用）：
      - 输入:  q: (B, Q, C), k: (B, K, C), v: (B, K, C)
      - 输出:  attn_out: (B, Q, C),  attn_weights(可选): (B, Q, K)  // 按 head 平均
    仅实现当前工程所需功能：
      - 支持 attn_mask∈{(Q,K),(B,Q,K),(B*H,Q,K)}，bool/float
      - 支持 key_padding_mask∈{(B,K)}，bool
      - 支持 dropout, bias, need_weights
      - 不支持：add_zero_attn、static_kv、不同的 batch_first 模式切换
    """

    def __init__(self,
                 embed_dim: int,
                 num_heads: int,
                 dropout: float = 0.0,
                 bias: bool = True):
        super().__init__()
        assert embed_dim % num_heads == 0, "embed_dim 必须能被 num_heads 整除"
        self.embed_dim = int(embed_dim)
        self.num_heads = int(num_heads)
        self.head_dim = self.embed_dim // self.num_heads
        self.scale = 1.0 / math.sqrt(self.head_dim)

        # 独立的 Q/K/V 投影与输出投影（更直观，少分支）
        self.q_proj = nn.Linear(self.embed_dim, self.embed_dim, bias=bias)
        self.k_proj = nn.Linear(self.embed_dim, self.embed_dim, bias=bias)
        self.v_proj = nn.Linear(self.embed_dim, self.embed_dim, bias=bias)
        self.out_proj = nn.Linear(self.embed_dim, self.embed_dim, bias=bias)

        self.attn_drop = nn.Dropout(dropout)

        # 参数初始化：对齐 PyTorch 常见策略
        self._reset_parameters()

    def _reset_parameters(self):
        # 类似 PyTorch 的 Xavier 初始化
        nn.init.xavier_uniform_(self.q_proj.weight)
        nn.init.xavier_uniform_(self.k_proj.weight)
        nn.init.xavier_uniform_(self.v_proj.weight)
        nn.init.xavier_uniform_(self.out_proj.weight)
        if self.q_proj.bias is not None:
            nn.init.zeros_(self.q_proj.bias)
            nn.init.zeros_(self.k_proj.bias)
            nn.init.zeros_(self.v_proj.bias)
            nn.init.zeros_(self.out_proj.bias)

    @staticmethod
    def _expand_attn_mask(attn_mask: torch.Tensor,
                          B: int,
                          H: int,
                          Q: int,
                          K: int,
                          device,
                          dtype) -> torch.Tensor:
        """
        统一把 mask 变成 (B, H, Q, K)，float 类型（被加到 scores 上）。
          - bool mask: True 表示不可见，会被置为 -inf
          - float mask: 直接加到 scores 上（例如含 -inf）
        允许形状：(Q,K) 或 (B,Q,K) 或 (B*H,Q,K)
        """
        if attn_mask.dtype == torch.bool:
            # True 为不可见 → -inf
            float_mask = torch.zeros_like(attn_mask, dtype=dtype)
            float_mask = float_mask.masked_fill(attn_mask, float("-inf"))
        else:
            float_mask = attn_mask.to(dtype)

        if float_mask.dim() == 2:
            # (Q, K) -> (1,1,Q,K) -> (B,H,Q,K)
            float_mask = float_mask.view(1, 1, Q, K).expand(B, H, Q, K)
        elif float_mask.dim() == 3:
            if float_mask.size(0) == B:
                # (B,Q,K) -> (B,1,Q,K) -> (B,H,Q,K)
                float_mask = float_mask.view(B, 1, Q, K).expand(B, H, Q, K)
            elif float_mask.size(0) == B * H:
                # (B*H,Q,K) -> (B,H,Q,K)（只改视图，不拷贝）
                float_mask = float_mask.view(B, H, Q, K)
            else:
                raise ValueError(f"attn_mask 第0维既不是 B({B}) 也不是 B*H({B * H})，实际 {float_mask.size(0)}")
        else:
            raise ValueError("attn_mask 维度必须为 2 或 3")
        return float_mask.to(device=device)

    @staticmethod
    def _apply_key_padding_mask(scores: torch.Tensor,
                                key_padding_mask: torch.Tensor):
        """
        scores: (B, H, Q, K)
        key_padding_mask: (B, K) bool，True 表示该 key 位置需要被屏蔽
        """
        if key_padding_mask is None:
            return scores
        if key_padding_mask.dtype != torch.bool:
            key_padding_mask = key_padding_mask.to(torch.bool)
        B, H, Q, K = scores.shape
        # (B,1,1,K) broadcast 到 (B,H,Q,K)
        mask = key_padding_mask.view(B, 1, 1, K)
        scores = scores.masked_fill(mask, float("-inf"))
        return scores

    def forward(self,
                query: torch.Tensor,  # (B, Q, C)
                key: torch.Tensor,  # (B, K, C)
                value: torch.Tensor,  # (B, K, C)
                attn_mask: Optional[torch.Tensor] = None,  # (Q,K) or (B,Q,K) or (B*H,Q,K)
                key_padding_mask: Optional[torch.Tensor] = None,  # (B, K) bool
                need_weights: bool = False,  # 返回平均后的注意力权重
                ) -> Tuple[torch.Tensor, Optional[torch.Tensor]]:

        B, Q, C = query.shape
        Bk, K, Ck = key.shape
        Bv, Kv, Cv = value.shape
        assert B == Bk == Bv, "q/k/v 的 batch 必须一致"
        assert C == Ck == Cv == self.embed_dim, "q/k/v 的通道必须等于 embed_dim"
        assert K == Kv, "key/value 的序列长度必须一致"

        # 1) 线性投影
        # qkv: (B, Q/K, C) → (B, Q/K, H*Dh) → reshape 为 (B, Q/K, H, Dh)
        q = self.q_proj(query).view(B, Q, self.num_heads, self.head_dim)
        k = self.k_proj(key).view(B, K, self.num_heads, self.head_dim)
        v = self.v_proj(value).view(B, K, self.num_heads, self.head_dim)

        # 2) 注意力分数（不做转置，直接用 einsum）
        # scores: (B, H, Q, K) = einsum('bqhd,bkhd->bhqk', q, k)
        scores = torch.einsum('bqhd,bkhd->bhqk', q, k) * self.scale  # 缩放点积

        # 3) 掩码
        if attn_mask is not None:
            mask_f = self._expand_attn_mask(attn_mask, B, self.num_heads, Q, K, scores.device, scores.dtype)
            scores = scores + mask_f  # (B,H,Q,K)

        if key_padding_mask is not None:
            scores = self._apply_key_padding_mask(scores, key_padding_mask)  # (B,H,Q,K)

        # 4) softmax & dropout
        attn = F.softmax(scores, dim=-1)  # (B,H,Q,K)
        attn = self.attn_drop(attn)

        # 5) 加权求和，得到上下文
        # context: (B, Q, H, Dh) = einsum('bhqk,bkhd->bqhd', attn, v)
        context = torch.einsum('bhqk,bkhd->bqhd', attn, v)

        # 6) 合并 heads → 输出线性
        context = context.reshape(B, Q, self.embed_dim)  # (B,Q,C)
        out = self.out_proj(context)  # (B,Q,C)

        if need_weights:
            # 返回按 head 平均的注意力权重: (B,Q,K)
            attn_weights = attn.mean(dim=1)  # (B,Q,K)
            return out, attn_weights
        else:
            return out, None


# =========================
# t-1 时序 Temporal-Cross-Attention
# =========================
class TemporalCrossAttention(nn.Module):
    """
    Sparse4D 风格（decouple_attn=True）t-1 跨注意力：
      - Q = concat(cur_q,  cur_pos)  → inner_dim
      - K = concat(prev_q, prev_pos) → inner_dim
      - V = fc_before(prev_q)        → inner_dim
      - out = fc_after(MHA(Q,K,V))   → embed_dim
    """

    def __init__(self,
                 embed_dim: int,  # C
                 pos_dim: int,  # P
                 *,
                 n_heads: int = 8,
                 dropout: float = 0.1):
        super().__init__()
        assert embed_dim > 0 and pos_dim > 0, "embed_dim 与 pos_dim 必须为正整数"
        inner_dim = embed_dim + pos_dim
        assert inner_dim % n_heads == 0, "inner_dim(=embed_dim+pos_dim) 必须能被 num_heads 整除"

        self.embed_dim = int(embed_dim)
        self.pos_dim = int(pos_dim)
        self.inner_dim = int(inner_dim)

        self.mha = FastBatchFirstMHA(
            embed_dim=self.inner_dim,
            num_heads=n_heads,
            dropout=dropout,
        )

        self.fc_before = nn.Linear(self.embed_dim, self.inner_dim, bias=False)
        self.fc_after = nn.Linear(self.inner_dim, self.embed_dim, bias=False)

    def forward(self,
                cur_q: torch.Tensor,  # (B,Q,C)
                cur_pos: torch.Tensor,  # (B,Q,P)
                prev_q: torch.Tensor,  # (B,K,C)
                prev_pos: torch.Tensor,  # (B,K,P)
                *,
                attn_mask: torch.Tensor = None  # (B*H, Q, K) 或 None
                ) -> torch.Tensor:
        # 解耦式：Q/K 拼内容与位置（不相加）
        q_in = torch.cat([cur_q, cur_pos], dim=-1)  # (B,Q,C+P)
        k_in = torch.cat([prev_q, prev_pos], dim=-1)  # (B,K,C+P)
        v_in = self.fc_before(prev_q)  # (B,K,C+P)
        out, _ = self.mha(q_in, k_in, v_in, attn_mask=attn_mask, need_weights=False)
        out = self.fc_after(out)  # (B,Q,C)
        return out


# =========================
# 自注意力
# =========================
class SelfAttention(nn.Module):
    """
    Sparse4D 风格自注意力（仅 decouple_attn=True）
    - Q, K = [q, pos]（拼接）
    - V    = fc_before(q): C -> C+P → fc_after -> C
    """

    def __init__(self,
                 embed_dim: int,
                 pos_dim: int,
                 n_heads: int = 6,
                 dropout: float = 0.1):
        super().__init__()
        assert embed_dim > 0 and pos_dim > 0
        inner_dim = embed_dim + pos_dim
        assert inner_dim % n_heads == 0

        self.embed_dim = int(embed_dim)
        self.pos_dim = int(pos_dim)
        self.inner_dim = int(inner_dim)

        self.mha = FastBatchFirstMHA(
            embed_dim=self.inner_dim,
            num_heads=n_heads,
            dropout=dropout,
        )
        self.fc_before = nn.Linear(self.embed_dim, self.inner_dim, bias=False)
        self.fc_after = nn.Linear(self.inner_dim, self.embed_dim, bias=False)

    def forward(self,
                q: torch.Tensor,  # (B, Q, C)
                pos: torch.Tensor,  # (B, Q, P)
                attn_mask: Optional[torch.Tensor] = None
                ) -> torch.Tensor:
        qk = torch.cat([q, pos], dim=-1)  # (B,Q,C+P)
        v = self.fc_before(q)  # (B,Q,C+P)
        out, _ = self.mha(qk, qk, v, attn_mask=attn_mask, need_weights=False)
        out = self.fc_after(out)  # (B,Q,C)
        return out


# =========================
# 世界点->图片grid
# =========================
class GridLoss(nn.Module):
    """
    grid 越界惩罚（配合 grid_sample 的 [-1,1] 坐标）
    - grid: (*, 2)
    - loss_mask: (*,) bool，True 表示参与 loss
    返回：标量 loss
    """

    def __init__(self, oob_thresh: float = 0.95):
        super().__init__()
        self.oob_thresh = float(oob_thresh)

    def forward(self, grid: torch.Tensor, loss_mask: torch.Tensor | None = None) -> torch.Tensor:
        assert grid.shape[-1] == 2, "grid 最后一维必须为 2"

        a = grid.abs() - self.oob_thresh  # (*,2)
        loss_xy = torch.tanh(F.relu(a)).mean(dim=-1)  # (*,)

        if loss_mask is None:
            mask = torch.ones(grid.shape[:-1], device=grid.device, dtype=torch.bool)
        else:
            mask = loss_mask.to(device=grid.device, dtype=torch.bool)
            assert mask.shape == grid.shape[:-1], f"loss_mask 形状应为 {grid.shape[:-1]}，实际 {mask.shape}"

        m = mask.to(grid.dtype)
        denom = m.sum().clamp_min(1.0)
        return (loss_xy * m).sum() / denom


class PointsToGrid(nn.Module):
    """
    世界点 -> grid_sample grid（只做投影，不计算损失）
    - 输入: world_pts (*,3)
    - 输出: grid (*,2)
    """

    def __init__(
            self,
            H: int, W: int, stride: int,
            K, R, t,
            align_corners: bool = True,
            eps: float = 0.01,
    ) -> None:
        super().__init__()
        self.H = int(H)
        self.W = int(W)
        self.s = int(stride)
        assert self.H % self.s == 0 and self.W % self.s == 0
        self.Hf = self.H // self.s
        self.Wf = self.W // self.s

        self.eps = float(eps)
        self.align_corners = bool(align_corners)

        K_t = torch.as_tensor(K, dtype=torch.float32)
        R_t = torch.as_tensor(R, dtype=torch.float32)
        t_t = torch.as_tensor(t, dtype=torch.float32).reshape(3)

        self.P = K_t @ R_t  # (3,3)
        self.tK = t_t @ K_t.T  # (3,)

        if self.align_corners:
            self.ax = 2.0 / ((self.Wf - 1.0) * self.s)
            self.bx = -1.0
            self.ay = 2.0 / ((self.Hf - 1.0) * self.s)
            self.by = -1.0
        else:
            self.ax = 2.0 / (self.Wf * self.s)
            self.bx = (1.0 / self.Wf) - 1.0
            self.ay = 2.0 / (self.Hf * self.s)
            self.by = (1.0 / self.Hf) - 1.0

    def forward(self, world_pts: torch.Tensor) -> torch.Tensor:
        assert world_pts.shape[-1] == 3, "最后一维必须为 3"

        P = self.P.to(device=world_pts.device, dtype=world_pts.dtype)
        tK = self.tK.to(device=world_pts.device, dtype=world_pts.dtype)

        pix_h = world_pts @ P.T + tK
        w = pix_h[..., 2].clamp_min(self.eps)

        x = pix_h[..., 0] / w
        y = pix_h[..., 1] / w

        grid_x = self.ax * x + self.bx
        grid_y = self.ay * y + self.by
        return torch.stack((grid_x, grid_y), dim=-1)  # (*,2)


# =========================
# 可学习关键点生成器
# =========================
class SampleKptsGenerator(nn.Module):
    """
    极致精简版：一次 bmm 完成局部→世界的旋转+平移（齐次坐标）
      - 固定7点 + L可学习点先合并为 (B,Q,T,3)
      - yaw_abs = yaw_rel (+ atan2(cy,cx) 若 rel_yaw=True)
      - world = [local, 1] @ Affine    （Affine 形如 [[Rz^T, 0],[t^T]]，但右乘 4x3）
    返回: (B,Q,T,3)
    """

    def __init__(self,
                 embed_dim: int,
                 num_learnable_pts: int = 7,
                 fix_scale=(
                         (0.0, 0.0, 0.0),  # center
                         (0.45, 0.0, 0.45),  # +x
                         (0.45, 0.0, -0.45),  # +x
                         (-0.45, 0.45, 0.0),  # -x
                         (-0.45, -0.45, 0.0),  # -x
                         (0.0, 0.45, 0.0),  # +y
                         (0.0, -0.45, 0.0),  # -y
                         (0.0, 0.0, 0.45),  # +z
                         (0.0, 0.0, -0.45),  # -z
                 ),
                 learnable_range: float = 1.0,
                 rel_yaw: bool = True,
                 eps: float = 1e-3):
        super().__init__()
        assert embed_dim > 0 and num_learnable_pts >= 0 and learnable_range > 0.0
        self.embed_dim = int(embed_dim)
        self.L = int(num_learnable_pts)
        self.learnable_range = float(learnable_range)
        self.rel_yaw = bool(rel_yaw)
        self.eps = float(eps)

        fs = torch.tensor(fix_scale, dtype=torch.float32)  # (M,3)
        assert fs.ndim == 2 and fs.shape[1] == 3
        self.register_buffer("fix_scale", fs, persistent=False)
        self.M = int(fs.shape[0])
        self.num_pts = self.M + self.L

        if self.L > 0:
            self.mlp = nn.Sequential(
                nn.Linear(self.embed_dim, self.embed_dim),
                nn.ReLU(inplace=True),
                nn.Linear(self.embed_dim, self.L * 3)
            )

    def forward(self, query: torch.Tensor, box: torch.Tensor) -> torch.Tensor:
        B, Q, C = query.shape
        assert box.shape == (B, Q, 8)
        dtype, device = box.dtype, box.device
        BQ = B * Q

        # 中心与尺寸
        ctr = box[..., 0:3]  # (B,Q,3)
        size = box[..., 3:6].clamp_min(self.eps)  # (B,Q,3)

        # ===== yaw 计算：避免使用 atan2，直接在 cos/sin 上做角度合成 =====
        # box[...,6:8] = (cos_rel, sin_rel) 相对视线角
        cos_rel = box[..., 6]
        sin_rel = box[..., 7]

        if self.rel_yaw:
            # 视线方向 phi 对应的 cos/sin：由 (cx,cy) 归一化得到
            cx = box[..., 0]
            cy = box[..., 1]
            # 半径 r，避免除零
            r = torch.sqrt(cx * cx + cy * cy + self.eps)

            cos_phi = cx / r
            sin_phi = cy / r

            # 绝对 yaw = yaw_rel + phi
            # 使用三角恒等式：
            # cos(a+b) = cos a cos b - sin a sin b
            # sin(a+b) = sin a cos b + cos a sin b
            cos_yaw = cos_rel * cos_phi - sin_rel * sin_phi
            sin_yaw = sin_rel * cos_phi + cos_rel * sin_phi
        else:
            # 直接使用 box 中存的相对角
            cos_yaw = cos_rel
            sin_yaw = sin_rel

        # 统一 reshape 成 (BQ,)
        c = cos_yaw.reshape(BQ)
        s = sin_yaw.reshape(BQ)

        # 齐次仿射矩阵 Affine:  (BQ,4,3)
        # [[ c,  s, 0],
        #  [-s,  c, 0],
        #  [ 0,  0, 1],
        #  [tx, ty, tz]]
        tx_ty_tz = ctr.reshape(BQ, 3)  # (BQ,3)
        Affine = torch.stack((
            torch.stack((c, s, torch.zeros_like(c)), dim=-1),
            torch.stack((-1 * s, c, torch.zeros_like(c)), dim=-1),
            torch.tensor([0.0, 0.0, 1.0], dtype=dtype, device=device).expand(BQ, -1),
            tx_ty_tz
        ), dim=1)  # (BQ,4,3)

        # 局部点：固定 + 可学习
        local_fix = (size.unsqueeze(2) *
                     self.fix_scale.to(dtype).view(1, 1, self.M, 3))  # (B,Q,M,3)

        if self.L > 0:
            off = self.mlp(query).view(B, Q, self.L, 3)  # (B,Q,L,3)
            off = torch.tanh(off) * (self.learnable_range * size.unsqueeze(2))  # (B,Q,L,3)
        else:
            off = box.new_zeros(B, Q, 0, 3)

        local = torch.cat([local_fix, off], dim=2).reshape(BQ, self.M + self.L, 3)  # (BQ,T,3)

        # 单次 bmm：齐次坐标右乘 Affine
        ones = torch.ones(BQ, local.size(1), 1, dtype=dtype, device=device)  # (BQ,T,1)
        local_h = torch.cat([local, ones], dim=-1)  # (BQ,T,4)
        world = torch.bmm(local_h, Affine)  # (BQ,T,3)
        return world.view(B, Q, self.M + self.L, 3)


class FixedSinCosTable(nn.Module):
    """
    固定身份编码表：返回形状 (N, D)，不做裁剪/补零。
    约束：D 必须为偶数（sin/cos 各 D/2 维）。
    """

    def __init__(self, num_tokens: int, dim: int):
        super().__init__()
        assert num_tokens > 0 and dim > 0
        assert dim % 2 == 0, f"FixedSinCosTable 要求 dim 为偶数，当前 dim={dim}"
        N, D = int(num_tokens), int(dim)
        half = D // 2

        k = torch.arange(N, dtype=torch.float32).unsqueeze(1)  # (N,1)
        i = torch.arange(half, dtype=torch.float32).unsqueeze(0)  # (1,half)
        w = 1.0 / (10000.0 ** (i / float(half)))  # (1,half)

        pe = torch.cat([torch.sin(k * w), torch.cos(k * w)], dim=1)  # (N, D)
        pe = F.normalize(pe, dim=1)
        self.register_buffer("table", pe, persistent=True)

    def forward(self) -> torch.Tensor:
        return self.table  # (N, D)


class SphericalFourierPE3D(nn.Module):
    """
    3D → P 的整体位置编码：一次 matmul + sin/cos。
    不做裁剪/补零；P 必须为偶数（输出恰好 P 维）。
    """

    def __init__(self, pos_dim: int, *, base: float = 10000.0):
        super().__init__()
        assert pos_dim > 0 and pos_dim % 2 == 0, f"pos_dim 必须为偶数，当前={pos_dim}"
        self.P = int(pos_dim)
        H = self.P // 2  # 2H = P

        # 对数频率
        i = torch.arange(H, dtype=torch.float32).unsqueeze(0)  # (1,H)
        w = base ** (-i / float(H))  # (1,H)

        # 方向：Fibonacci 球面采样
        j = torch.arange(H, dtype=torch.float32)
        phi = 2.0 * math.pi * (j / ((1.0 + 5.0 ** 0.5) / 2.0))  # [0,2π)
        z = 2.0 * (j + 0.5) / H - 1.0  # [-1,1]
        r = torch.sqrt((1.0 - z * z).clamp_min(0.0))
        dirs = torch.stack([r * torch.cos(phi), r * torch.sin(phi), z], dim=0)  # (3,H)
        dirs = F.normalize(dirs, dim=0)  # 列单位化

        proj = dirs * w  # (3,H)
        self.register_buffer("proj", proj, persistent=True)

    def forward(self, pts: torch.Tensor) -> torch.Tensor:
        """
        pts: (*, 3) -> (*, P)
        """
        assert pts.shape[-1] == 3, f"最后一维需要是3，当前={pts.shape[-1]}"
        proj = self.proj.to(dtype=pts.dtype, device=pts.device)  # (3,H)
        phi = torch.matmul(pts, proj)  # (*,H)
        pe = torch.cat([torch.sin(phi), torch.cos(phi)], dim=-1)  # (*,2H=P)
        return pe


# ===== 新增：固定几何位置编码 =====
class BoxFourierPE8D(nn.Module):
    """
    8D 盒子整体位置编码（极简版）：
      输入: (..., 8) = [cx, cy, cz, l, w, h, cos, sin]
      输出: (..., pos_dim), 其中 pos_dim 必须为偶数 (pos_dim = 2*H)

    做法:
      - 构造一个固定的 8×H 投影矩阵 proj（列向量单位化），再乘以对数频率权重 w
      - φ = box @ proj
      - 输出 concat(sin φ, cos φ)
    备注:
      - 不做 gz，不取对数，不做裁剪/补零
      - 提供一个 8 维 scale（可选）用于粗量纲对齐，默认全 1
    """

    def __init__(self,
                 pos_dim: int,
                 *,
                 base: float = 10000.0,
                 scale: torch.Tensor | None = None):
        super().__init__()
        assert pos_dim > 0 and pos_dim % 2 == 0, f"pos_dim 必须为偶数，当前={pos_dim}"
        self.P = int(pos_dim)
        self.H = self.P // 2

        # 对数间隔频率权重 (H,)
        i = torch.arange(self.H, dtype=torch.float32)
        w = base ** (-i / float(self.H))  # (H,)

        # 8D 方向列向量（确定性、可复现；列归一化）
        j = torch.arange(self.H, dtype=torch.float32)  # (H,)
        d = torch.arange(1, 9, dtype=torch.float32).unsqueeze(1)  # (8,1)
        golden = 2.0 * math.pi / ((1.0 + 5.0 ** 0.5) / 2.0)  # ~ 2π/φ
        angles = d * (j + 1.0) * golden  # (8,H)
        dirs = torch.sin(angles)  # (8,H)
        dirs = F.normalize(dirs, dim=0)  # 列单位化

        # 投影矩阵: 每列方向 × 频率权重
        proj = dirs * w  # (8,H)
        self.register_buffer("proj", proj, persistent=True)

        # 简单量纲缩放（可选）
        if scale is None:
            scale = torch.ones(8, dtype=torch.float32)
        else:
            scale = torch.as_tensor(scale, dtype=torch.float32).view(8)
        self.register_buffer("scale", scale, persistent=True)

    def forward(self, box: torch.Tensor) -> torch.Tensor:
        assert box.shape[-1] == 8, f"最后一维必须为 8，当前={box.shape[-1]}"
        dtype, device = box.dtype, box.device
        b = box * self.scale.to(dtype=dtype, device=device)  # (..., 8)
        phi = torch.matmul(b, self.proj.to(dtype=dtype, device=device))  # (..., H)
        return torch.cat([torch.sin(phi), torch.cos(phi)], dim=-1)  # (..., 2H = P)


# =========================
# 单视角采样注入器（替换原多视角版本）
# =========================
class SingleViewSamplerInject(nn.Module):
    """
    单视角采样注入器（无 teacher 逻辑版）：
      - 输入
          q:       (B, Q, C)
          pos_q:   (B, Q, P)
          pts:     (B, Q, K, 3)  世界坐标（K=7+L，index 0 为中心点）
          feats_b: (B, C, Hf, Wf)
      - 训练：仅对中心点参与几何正则；采样仍用全部 K 个点
    """

    def __init__(self,
                 embed_dim: int,
                 pos_dim: int,
                 expected_kpts: int,
                 n_heads: int = 6,
                 *,
                 dropout: float = 0.1,
                 align_corners: bool = True,
                 eps: float = 1e-3):
        super().__init__()
        assert embed_dim > 0 and pos_dim > 0
        assert (embed_dim + pos_dim) % n_heads == 0
        assert embed_dim % 2 == 0, f"为了无裁剪/补零，embed_dim 建议为偶数，当前={embed_dim}"

        self.embed_dim = int(embed_dim)
        self.pos_dim = int(pos_dim)
        self.inner_dim = self.embed_dim + self.pos_dim
        self.expected_kpts = int(expected_kpts)

        self.align_corners = bool(align_corners)
        self.eps = float(eps)

        self.ptg: Optional[PointsToGrid] = None
        self._calib_ready = False

        # 注意力与通道投影
        self.mha = FastBatchFirstMHA(embed_dim=self.inner_dim, num_heads=n_heads, dropout=dropout)
        self.fc_before = nn.Linear(self.embed_dim, self.inner_dim, bias=False)
        self.fc_after = nn.Linear(self.inner_dim, self.embed_dim, bias=False)

        # 抽取后的组件
        self.kpt_id = FixedSinCosTable(self.expected_kpts, self.embed_dim)  # (K, C)
        self.pe3d = SphericalFourierPE3D(self.pos_dim)  # (*, P)

    @torch.no_grad()
    def set_calibration(self, H, W, stride, K, R, t):
        self.ptg = PointsToGrid(H=H, W=W, stride=stride, K=K, R=R, t=t)
        self._calib_ready = True

    def forward(self,
                q: torch.Tensor,  # (B, Q, C)
                pos_q: torch.Tensor,  # (B, Q, P)
                pts: torch.Tensor,  # (B, Q, K, 3)
                feats_b: torch.Tensor,  # (B, C, Hf, Wf)
                ):
        assert self._calib_ready and (self.ptg is not None), "未设置标定, 请先调用 set_calibration(...)"

        B, Q, C = q.shape
        VB, Cx, Hf, Wf = feats_b.shape
        assert VB == B and Cx == C == self.embed_dim
        Kpts = int(pts.size(2))
        if self.expected_kpts > 0:
            assert self.expected_kpts == Kpts, f"expected_kpts={self.expected_kpts}, got {Kpts}"

        # === 1) 世界点 -> 采样网格 ===
        grid = self.ptg(pts)  # (B,Q,K,2), dict

        # === 2) 特征采样 ===
        sampled = F.grid_sample(
            feats_b, grid, mode='bilinear',
            padding_mode='zeros',  # 与原实现保持一致；如需统一可改为 'border'
            align_corners=self.align_corners
        )  # (B, C, Q, K)
        sampled = sampled.permute(0, 2, 3, 1).contiguous()  # (B, Q, K, C)

        # === 3) 关键点身份编码 ===
        Ek = self.kpt_id().to(dtype=sampled.dtype, device=sampled.device)  # (K, C)
        Ek = Ek.view(1, 1, Kpts, C).expand(B, Q, -1, -1)  # (B, Q, K, C)
        sampled_with_id = sampled + Ek

        # === 4) 关键点 3D 位置编码 ===
        pos_k = self.pe3d(pts)  # (B, Q, K, P)

        # === 5) 组装 Q/K/V 并 Cross-Attn ===
        q_in = torch.cat([q, pos_q], dim=-1)  # (B, Q, C+P)
        k_in = torch.cat([sampled_with_id, pos_k], dim=-1)  # (B, Q, K, C+P)
        v_in = self.fc_before(sampled)  # (B, Q, K, C+P)

        BQ = B * Q
        out, _ = self.mha(
            q_in.view(BQ, 1, self.inner_dim),
            k_in.view(BQ, Kpts, self.inner_dim),
            v_in.view(BQ, Kpts, self.inner_dim),
            need_weights=False
        )
        fused = self.fc_after(out.view(B, Q, self.inner_dim))  # (B, Q, C)

        return (fused, grid) if self.training else fused


# =========================
# GroundHead **2D**：z(x,y) 二维地形（单视角，Cross-Attn）
# =========================
# class GroundHead2D(nn.Module):
#     def __init__(self,
#                  embed_dim: int,
#                  pos_dim: int,
#                  sample_x: Iterable[float],
#                  sample_y: Iterable[float],
#                  x_offsets: Iterable[float] = (-3, 0, 3),
#                  y_offsets: Iterable[float] = (-3, 0, 3),
#                  n_heads: int = 6,
#                  dropout: float = 0.1,
#                  align_corners: bool = True,
#                  eps: float = 1e-3,
#                  scale_max: float = 3.0):
#         super().__init__()
#         assert embed_dim > 0 and pos_dim > 0
#         assert (embed_dim + pos_dim) % n_heads == 0
#         assert embed_dim % 2 == 0, f"为了无裁剪/补零，embed_dim 建议为偶数，当前={embed_dim}"
#         assert pos_dim % 2 == 0, f"为了无裁剪/补零，pos_dim 必须为偶数，当前={pos_dim}"
#
#         self.C = int(embed_dim)
#         self.P = int(pos_dim)
#         self.inner_dim = self.C + self.P
#         self.n_heads = int(n_heads)
#         self.align_corners = bool(align_corners)
#         self.eps = float(eps)
#         self.scale_max = float(scale_max)
#
#         sx = torch.as_tensor(list(sample_x), dtype=torch.float32)
#         sy = torch.as_tensor(list(sample_y), dtype=torch.float32)
#         dx = torch.as_tensor(list(x_offsets), dtype=torch.float32)
#         dy = torch.as_tensor(list(y_offsets), dtype=torch.float32)
#         self.register_buffer("sample_x", sx, persistent=True)
#         self.register_buffer("sample_y", sy, persistent=True)
#         self.register_buffer("x_offsets", dx, persistent=True)
#         self.register_buffer("y_offsets", dy, persistent=True)
#
#         self.Mx, self.My = int(sx.numel()), int(sy.numel())
#         self.K, self.L = int(dx.numel()), int(dy.numel())
#         self.M, self.T = self.Mx * self.My, self.K * self.L
#
#         self.q_table = nn.Parameter(torch.empty(self.M, self.C))
#         nn.init.trunc_normal_(self.q_table, std=0.02)
#
#         self.mha = FastBatchFirstMHA(
#             embed_dim=self.inner_dim, num_heads=n_heads, dropout=dropout
#         )
#         self.fc_before = nn.Linear(self.C, self.inner_dim, bias=False)
#         self.fc_after = nn.Linear(self.inner_dim, self.C, bias=False)
#
#         # 抽取后的组件
#         self.tok_id = FixedSinCosTable(self.T, self.C)  # (T,C)
#         self.pe3d = SphericalFourierPE3D(self.P)  # (*,P)
#
#         self.depth_head = nn.Sequential(
#             nn.Linear(self.C, self.C), nn.ReLU(inplace=True),
#             nn.Linear(self.C, 1)
#         )
#         for m in self.depth_head.modules():
#             if isinstance(m, nn.Linear):
#                 nn.init.kaiming_normal_(m.weight, nonlinearity='relu')
#                 if m.bias is not None: nn.init.zeros_(m.bias)
#
#         self._calib_ready = False
#         self.ptg: Optional[PointsToGrid] = None
#         self.register_buffer("grid_ctx", torch.empty(0), persistent=False)  # (M,T,2)
#         self.register_buffer("pos_xy_pe", torch.empty(0), persistent=False)  # (M,P)
#         self.register_buffer("pos_ctx_pe", torch.empty(0), persistent=False)  # (M,T,P)
#
#     @torch.no_grad()
#     def set_calibration(self, H, W, stride, K, R, t):
#
#         self.ptg = PointsToGrid(H=H, W=W, stride=stride, K=K, R=R, t=t)
#         self._calib_ready = True
#
#         device = K.device
#         sx = self.sample_x.to(device=device)
#         sy = self.sample_y.to(device=device)
#         dx = self.x_offsets.to(device=device)
#         dy = self.y_offsets.to(device=device)
#
#         x_min, x_max = sx.min(), sx.max()
#         scale = torch.ones_like(sx) if float(x_max - x_min) < 1e-6 else \
#             (1.0 + (sx - x_min) / (x_max - x_min) * (self.scale_max - 1.0))
#         dx_s = dx.view(1, self.K) * scale.view(self.Mx, 1)
#         dy_s = dy.view(1, self.L) * scale.view(self.My, 1)
#
#         Xc = sx.view(self.Mx, 1, 1, 1) + dx_s.view(self.Mx, self.K, 1, 1)
#         Yc = sy.view(1, 1, self.My, 1).expand(self.Mx, self.K, self.My, 1) + dy_s.view(self.Mx, 1, 1, self.L)
#         Zc = torch.zeros_like(Yc)
#         P_ctx = torch.stack([Xc.expand(self.Mx, self.K, self.My, self.L), Yc, Zc], dim=-1) \
#             .permute(0, 2, 1, 3, 4).contiguous()  # (Mx,My,K,L,3)
#
#         X0 = sx.view(self.Mx, 1).expand(self.Mx, self.My)
#         Y0 = sy.view(1, self.My).expand(self.Mx, self.My)
#         Z0 = torch.zeros_like(X0)
#         P_ctr = torch.stack([X0, Y0, Z0], dim=-1)  # (Mx,My,3)
#
#         grid_ctx = self.ptg(P_ctx).reshape(self.M, self.T, 2).detach()
#         pos_xy_pe = self.pe3d(P_ctr).reshape(self.M, self.P).detach()
#         pos_ctx_pe = self.pe3d(P_ctx).reshape(self.M, self.T, self.P).detach()
#
#         self.grid_ctx = grid_ctx
#         self.pos_xy_pe = pos_xy_pe
#         self.pos_ctx_pe = pos_ctx_pe
#
#     def forward(self, feats_b: torch.Tensor) -> torch.Tensor:
#         assert self._calib_ready and self.grid_ctx.numel() > 0
#         B, C, Hf, Wf = feats_b.shape
#         assert C == self.C
#         M, T = self.M, self.T
#         dtype, device = feats_b.dtype, feats_b.device
#
#         grid_ctx = self.grid_ctx.to(device=device, dtype=dtype).view(1, M, T, 2).expand(B, M, T, 2)
#         sampled = F.grid_sample(
#             feats_b, grid_ctx, mode='bilinear',
#             padding_mode='zeros',
#             align_corners=self.align_corners
#         )  # (B,C,M,T)
#         tokens = sampled.permute(0, 2, 3, 1).contiguous()  # (B,M,T,C)
#
#         tok_id = self.tok_id().to(device=device, dtype=dtype).view(1, 1, T, self.C).expand(B, M, -1, -1)
#         tokens_with_id = tokens + tok_id  # (B,M,T,C)
#
#         pos_xy = self.pos_xy_pe.to(device=device, dtype=dtype).view(1, M, self.P).expand(B, M, self.P)
#         pos_ctx = self.pos_ctx_pe.to(device=device, dtype=dtype).view(1, M, T, self.P).expand(B, M, T, self.P)
#
#         q = self.q_table.to(device=device, dtype=dtype).view(1, M, self.C).expand(B, M, self.C)
#         q_in = torch.cat([q, pos_xy], dim=-1)
#         k_in = torch.cat([tokens_with_id, pos_ctx], dim=-1)
#         v_in = self.fc_before(tokens_with_id)
#
#         BM = B * M
#         out, _ = self.mha(
#             q_in.view(BM, 1, self.inner_dim),
#             k_in.view(BM, T, self.inner_dim),
#             v_in.view(BM, T, self.inner_dim),
#             need_weights=False
#         )
#         fused = self.fc_after(out.view(B, M, self.inner_dim))  # (B,M,C)
#
#         dz = self.depth_head(fused).squeeze(-1)  # (B,M)
#         return dz.view(B, self.Mx, self.My)


class BoxProposalHead(nn.Module):
    """
    3D Box Proposal（固定物理采样；不回归 dxyz）

    - 中心 = sample_xyz（固定，不回归）
    - 仅预测尺寸 (l,w,h) 与朝向 (cos,sin)
    - 输出分类 logits（含背景 0，维度为 num_classes+1）

    返回（注意顺序）：
      - 当 topk <= 0（不做 Top-K）：
          (prop_box, query, cls_logits)
            prop_box:  (B, N, 8)  # [cx,cy,cz,l,w,h,cos,sin]，中心来自 sample_xyz
            query:     (B, N, C)  # 对应 proposal 的 query 特征
            cls_logits:(B, N, C+1)# 分类 logits（含背景0）

      - 当 topk > 0（做 Top-K）：
          (query_topk, prop_box_topk, cls_logits_topk)
            query_topk:     (B, K, C)
            prop_box_topk:  (B, K, 8)
            cls_logits_topk:(B, K, C+1
    """

    def __init__(self,
                 sample_xyz,  # (N,3) 或 (1,N,3) 或 list/np
                 topk,  # 固定 Top-K；<=0 表示不做 Top-K
                 embed_dim: int,
                 pos_dim: int,
                 num_classes: int,  # 不含背景类的类别数；内部会输出 C+1（含背景0）
                 x_offsets=(-3, 0, 3),
                 y_offsets=(-3, 0, 3),
                 z_offsets=(-1.0, 1.0),
                 n_heads: int = 6,
                 dropout: float = 0.1,
                 align_corners: bool = True,
                 eps: float = 1e-3,
                 min_size: float = 1e-3,
                 ):
        super().__init__()
        assert embed_dim > 0 and pos_dim > 0
        assert (embed_dim + pos_dim) % n_heads == 0
        assert pos_dim % 2 == 0, "pos_dim 必须为偶数"
        assert embed_dim % 2 == 0, "embed_dim 建议为偶数"
        assert num_classes >= 1, "num_classes 至少为 1"

        self.C = int(embed_dim)
        self.P = int(pos_dim)
        self.inner = self.C + self.P
        self.align_corners = bool(align_corners)
        self.eps = float(eps)
        self.min_size = float(min_size)
        self.topk = int(topk) if topk is not None else 0
        self.num_classes = int(num_classes)
        self.num_out = self.num_classes + 1  # 含背景0

        # 3D 邻域偏移（仅用于构造上下文 tokens）
        dx = torch.as_tensor(list(x_offsets), dtype=torch.float32)
        dy = torch.as_tensor(list(y_offsets), dtype=torch.float32)
        dz = torch.as_tensor(list(z_offsets), dtype=torch.float32)
        self.register_buffer("x_offsets", dx, persistent=True)
        self.register_buffer("y_offsets", dy, persistent=True)
        self.register_buffer("z_offsets", dz, persistent=True)
        self.K = int(dx.numel())
        self.L = int(dy.numel())
        self.Mz = int(dz.numel())
        self.T = self.K * self.L * self.Mz  # 上下文 token 数

        # 注意力（decouple: Q=[q_feat,pos], K=[ctx_tok,pos], V=proj(ctx_tok)）
        self.mha = FastBatchFirstMHA(embed_dim=self.inner, num_heads=n_heads, dropout=dropout)
        self.fc_before = nn.Linear(self.C, self.inner, bias=False)  # V: C -> C+P
        self.fc_after = nn.Linear(self.inner, self.C, bias=False)  # 输出回 C

        # 位置编码 / token 身份
        self.pe3d = SphericalFourierPE3D(self.P)
        self.tok_id = FixedSinCosTable(self.T, self.C)

        # 预测头：只预测尺寸与朝向（不回归 dxyz）
        self.pred_head = nn.Sequential(
            nn.Linear(self.C, self.C), nn.ReLU(True),
            nn.Linear(self.C, 5)  # [l,w,h, cos, sin]
        )
        # 分类头（logits，含背景0）
        self.cls_head = nn.Sequential(
            nn.Linear(self.C, self.C), nn.ReLU(True),
            nn.Linear(self.C, self.num_out)
        )
        for m in list(self.pred_head.modules()) + list(self.cls_head.modules()):
            if isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(m.weight, nonlinearity='relu')
                if m.bias is not None: nn.init.zeros_(m.bias)

        # 投影器 & 缓存
        self.ptg: Optional[PointsToGrid] = None
        self._calib_ready = False

        # 样本点（init 固定）与派生缓存（在 set_calibration 自动计算）
        s = torch.as_tensor(sample_xyz, dtype=torch.float32)
        if s.dim() == 3:
            assert s.size(0) == 1, "sample_xyz 仅支持 (N,3) 或 (1,N,3)"
            s = s[0]
        assert s.dim() == 2 and s.size(-1) == 3, "sample_xyz 形状需为 (N,3)"
        self.register_buffer("sample_xyz", s.contiguous(), persistent=True)  # (N,3)

        self.register_buffer("grid_ctx", torch.empty(0), persistent=False)  # (N,T,2)
        self.register_buffer("pos_center", torch.empty(0), persistent=False)  # (N,P)
        self.register_buffer("pos_ctx", torch.empty(0), persistent=False)  # (N,T,P)
        self.register_buffer("grid_ctr", torch.empty(0), persistent=False)  # (N,2)
        self._cache_ready = False

    @torch.no_grad()
    def set_calibration(self, H, W, stride, K, R, t):
        """相机/特征尺度初始化，并自动基于 sample_xyz 预计算上下文与中心采样缓存"""

        self.ptg = PointsToGrid(H=H, W=W, stride=stride, K=K, R=R, t=t)
        self._calib_ready = True

        # ---- 自动构建缓存 ----
        dtype = self.ptg.P.dtype
        device = self.ptg.P.device
        s = self.sample_xyz.to(dtype=dtype, device=device)  # (N,3)
        N = int(s.size(0))

        dx = self.x_offsets.to(device=device, dtype=dtype)
        dy = self.y_offsets.to(device=device, dtype=dtype)
        dz = self.z_offsets.to(device=device, dtype=dtype)

        # 邻域 3D 点 (N,T,3)
        X = s[:, 0].view(N, 1, 1, 1) + dx.view(1, self.K, 1, 1)
        Y = s[:, 1].view(N, 1, 1, 1) + dy.view(1, 1, self.L, 1)
        Z = s[:, 2].view(N, 1, 1, 1) + dz.view(1, 1, 1, self.Mz)
        P_ctx = torch.stack([
            X.expand(N, self.K, self.L, self.Mz),
            Y.expand(N, self.K, self.L, self.Mz),
            Z.expand(N, self.K, self.L, self.Mz)
        ], dim=-1).view(N, self.T, 3)

        # 预计算采样网格与位置编码
        grid_ctx = self.ptg(P_ctx).to(torch.float32)  # (N,T,2)
        grid_ctr = self.ptg(s).to(torch.float32)  # (N,2)
        pos_center = self.pe3d(s).to(torch.float32)  # (N,P)
        pos_ctx = self.pe3d(P_ctx).to(torch.float32)  # (N,T,P)

        self.grid_ctx = grid_ctx.detach()
        self.grid_ctr = grid_ctr.detach()
        self.pos_center = pos_center.detach()
        self.pos_ctx = pos_ctx.detach()
        self._cache_ready = True

    def forward(self, feats_b: torch.Tensor):
        assert self._calib_ready and self._cache_ready, "请先调用 set_calibration(...) 完成缓存初始化"
        B, C, _, _ = feats_b.shape

        assert C == self.C
        dtype, device = feats_b.dtype, feats_b.device

        N = self.sample_xyz.size(0)

        sample = self.sample_xyz.to(device=device, dtype=dtype).unsqueeze(0).expand(B, -1, -1)  # (B,N,3)
        grid_ctx = self.grid_ctx.to(device=device, dtype=dtype).unsqueeze(0).expand(B, -1, -1, -1)  # (B,N,T,2)
        grid_ctr = self.grid_ctr.to(device=device, dtype=dtype).unsqueeze(0).unsqueeze(2).expand(B, -1, 1,
                                                                                                 -1)  # (B,N,1,2)
        pos_center = self.pos_center.to(device=device, dtype=dtype).unsqueeze(0).expand(B, -1, -1)  # (B,N,P)
        pos_ctx = self.pos_ctx.to(device=device, dtype=dtype).unsqueeze(0).expand(B, -1, -1, -1)  # (B,N,T,P)

        sampled = F.grid_sample(
            feats_b, grid_ctx, mode="bilinear",
            padding_mode="zeros", align_corners=self.align_corners
        )  # (B,C,N,T)
        tokens = sampled.permute(0, 2, 3, 1)  # (B,N,T,C)

        tok_id = self.tok_id().to(device=device, dtype=dtype).unsqueeze(0).unsqueeze(0)  # (1,1,T,C)
        tok_id = tok_id.expand(B, N, -1, -1)  # (B,N,T,C)
        tokens_with_id = tokens + tok_id

        center_feat = F.grid_sample(
            feats_b, grid_ctr, mode="bilinear",
            padding_mode="zeros", align_corners=self.align_corners
        )  # (B,C,N,1)
        q_feat = center_feat.squeeze(-1).permute(0, 2, 1)  # (B,N,C)

        q_in = torch.cat([q_feat, pos_center], dim=-1)  # (B,N,inner)
        k_in = torch.cat([tokens_with_id, pos_ctx], dim=-1)  # (B,N,T,inner)
        v_in = self.fc_before(tokens_with_id)  # (B,N,T,inner)

        BN = B * N
        out, _ = self.mha(
            q_in.reshape(BN, 1, self.inner),
            k_in.reshape(BN, self.T, self.inner),
            v_in.reshape(BN, self.T, self.inner),
            need_weights=False,
        )
        out = out.reshape(B, N, self.inner)
        fused = self.fc_after(out)  # (B,N,C)

        raw = self.pred_head(fused)  # (B,N,5)
        lwh, cs = raw.split((3, 2), dim=-1)
        size = F.softplus(lwh) + self.min_size
        cs = F.normalize(cs, dim=-1, eps=1e-6)
        prop_box = torch.cat([sample, size, cs], dim=-1)  # (B,N,8)

        cls_logits = self.cls_head(fused)  # (B,N,C+1)

        if (self.topk is None) or (self.topk <= 0) or (prop_box.size(1) == 0):
            return prop_box, fused, cls_logits

        prob = torch.softmax(cls_logits, dim=-1)
        if self.num_out >= 2:
            score_fg = prob[..., 1:].max(dim=-1).values  # (B,N)
        else:
            score_fg = prob[..., 0]

        K = min(int(self.topk), prop_box.size(1))
        topi = torch.topk(score_fg, k=K, dim=1).indices  # (B,K)

        idx_box = topi.unsqueeze(-1).expand(-1, -1, prop_box.size(-1))
        idx_fused = topi.unsqueeze(-1).expand(-1, -1, fused.size(-1))
        idx_cls = topi.unsqueeze(-1).expand(-1, -1, cls_logits.size(-1))

        box_topk = torch.gather(prop_box, dim=1, index=idx_box)
        query_topk = torch.gather(fused, dim=1, index=idx_fused)
        cls_logits_topk = torch.gather(cls_logits, dim=1, index=idx_cls)

        return query_topk, box_topk, cls_logits_topk


class Refiner(nn.Module):
    """
    Refine（box_head=7维）：center/size 加性残差；yaw 回归标量 Δyaw，再旋转更新 (cos,sin)

    box_head 输出 7 维：
      [d_cx, d_cy, d_cz, d_l, d_w, d_h, d_yaw]

    residual_mask 支持：
      - 长度 7：最后一维控制 yaw
      - 长度 8：若 mask[6] 或 mask[7] 为 1，则 yaw 启用（合并为 1 个 yaw mask）
    """

    def __init__(self,
                 embed_dim: int,
                 pos_dim: int,
                 num_classes: int,  # 含背景0
                 min_size: float = 1e-4,
                 residual_mask: Iterable[int] | torch.Tensor = (1, 1, 1, 1, 1, 1, 1, 1),
                 max_delta_yaw: float = math.pi):
        super().__init__()
        self.C = int(embed_dim)
        self.P = int(pos_dim)
        self.in_dim = self.C + self.P
        self.num_classes = int(num_classes)
        self.min_size = float(min_size)
        self.max_delta_yaw = float(max_delta_yaw)

        # ===== box_head 改为 7 维 =====
        self.box_head = nn.Sequential(
            nn.Linear(self.in_dim, self.C), nn.ReLU(inplace=True),
            nn.Linear(self.C, 7)
        )
        self.cls_head = nn.Sequential(
            nn.Linear(self.in_dim, self.C), nn.ReLU(inplace=True),
            nn.Linear(self.C, self.num_classes)
        )

        # ===== residual_mask 规范化为 7 维 =====
        if residual_mask is None:
            mask7 = torch.ones(7, dtype=torch.float32)
        else:
            m = torch.as_tensor(residual_mask, dtype=torch.float32).flatten()
            m = (m > 0.5).to(torch.float32)
            if m.numel() == 7:
                mask7 = m
            elif m.numel() == 8:
                # 8维的 cos/sin mask 合并成 1 个 yaw mask
                yaw_m = (m[6] + m[7]).clamp_max(1.0)
                mask7 = torch.cat([m[0:6], yaw_m.view(1)], dim=0)
            else:
                raise ValueError(f"residual_mask length must be 7 or 8, got {m.numel()}")

        self.residual_mask = mask7  # (7,)

    def forward(self,
                q: torch.Tensor,  # (B,Q,C)
                pos: torch.Tensor,  # (B,Q,P)
                box: torch.Tensor  # (B,Q,8) = [cx,cy,cz,l,w,h,cos,sin]
                ):
        x = torch.cat([q, pos], dim=-1)  # (B,Q,C+P)
        delta = self.box_head(x)  # (B,Q,7)
        cls_logits = self.cls_head(x)  # (B,Q,num_classes)

        m = self.residual_mask.to(dtype=delta.dtype, device=delta.device).view(1, 1, 7)

        # ===== center + size =====
        base_ctr = box[..., 0:3]
        base_size = box[..., 3:6]

        d_ctr = delta[..., 0:3] * m[..., 0:3]
        d_size = delta[..., 3:6] * m[..., 3:6]

        new_ctr = base_ctr + d_ctr
        new_size = (base_size + d_size).clamp_min(self.min_size)

        # ===== yaw: 标量 Δyaw 旋转更新 =====
        cs_base = box[..., 6:8]
        cs_base = cs_base / cs_base.norm(dim=-1, keepdim=True).clamp_min(1e-6)

        yaw_on = (m[..., 6] > 0.0)  # (1,1) bool，mask固定广播
        if torch.all(~yaw_on):
            cs = cs_base
        else:
            d_yaw_raw = delta[..., 6] * m[..., 6]  # (B,Q)
            d_yaw = self.max_delta_yaw * torch.tanh(d_yaw_raw)  # 限幅更稳

            cos_d = torch.cos(d_yaw)
            sin_d = torch.sin(d_yaw)

            cos0 = cs_base[..., 0]
            sin0 = cs_base[..., 1]

            cos_new = cos0 * cos_d - sin0 * sin_d
            sin_new = sin0 * cos_d + cos0 * sin_d
            cs_rot = torch.stack([cos_new, sin_new], dim=-1)

            cs = torch.where(yaw_on.unsqueeze(-1), cs_rot, cs_base)

        new_box = torch.cat([new_ctr, new_size, cs], dim=-1)  # (B,Q,8)
        return new_box, cls_logits


class DetDecoderLayer(nn.Module):
    """
    解码层（train / predict 两条路径分开），已移除地面建模依赖：
      A) （已删除）采样前“地面对齐”步骤
      B) （train-only）teacher 辅助：匈牙利匹配 + 几何/关键点混合
      C) 注意力：Temporal（可选）→ Self（可选）
      D) 关键点 → 单视角 Cross-Inject（train 会返回 grid）
      E) Refiner（不再二次地面对齐，是否更新 cz 由 residual_mask 决定）
    """

    def __init__(self,
                 embed_dim: int,
                 pos_dim: int,
                 num_classes: int,
                 *,
                 n_heads: int = 6,
                 dropout: float = 0.0,
                 num_learnable_pts: int = 7,
                 use_temporal_attn: bool = True,
                 use_self_attn: bool = True):
        super().__init__()
        self.embed_dim = int(embed_dim)
        self.pos_dim = int(pos_dim)
        self.use_temporal_attn = bool(use_temporal_attn)
        self.use_self_attn = bool(use_self_attn)

        # 关键几何超参（与损失/匹配保持一致）
        self.norm_eps = 1e-3
        self.min_size = 1e-2
        self.center_on_ground_alpha = 0.5  # α（仅用于损失/teacher混合中的 g_z 计算，不再用于 zmap 对齐）
        self.teacher_ncd_thresh = 2.5

        # 公用匹配器
        self.matcher = Hungarian3DMatcher()

        # 几何位置编码（8D box → P）
        self.box_pe = BoxFourierPE8D(
            pos_dim=self.pos_dim,
            scale=[1 / 80, 1 / 20, 1 / 4, 1 / 6, 1 / 6, 1 / 6, 1, 1]
        )

        # 注意力与采样模块
        self.temporal_attn = TemporalCrossAttention(
            embed_dim=self.embed_dim, pos_dim=self.pos_dim,
            n_heads=n_heads, dropout=dropout
        )
        self.self_attn = SelfAttention(
            embed_dim=self.embed_dim, pos_dim=self.pos_dim,
            n_heads=n_heads, dropout=dropout
        )
        self.kpt_gen = SampleKptsGenerator(
            embed_dim=self.embed_dim,
            num_learnable_pts=num_learnable_pts
        )
        self.cross_inject = SingleViewSamplerInject(
            embed_dim=self.embed_dim, pos_dim=self.pos_dim,
            n_heads=n_heads, dropout=dropout,
            expected_kpts=9 + num_learnable_pts, align_corners=True
        )

        # FFN + LayerNorm
        self.ffn = nn.Sequential(
            nn.Linear(embed_dim, self.embed_dim * 2), nn.ReLU(inplace=True),
            nn.Linear(self.embed_dim * 2, self.embed_dim)
        )
        self.layernorm1 = nn.LayerNorm(self.embed_dim)
        self.layernorm2 = nn.LayerNorm(self.embed_dim)
        self.layernorm3 = nn.LayerNorm(self.embed_dim)

        self.refiner = Refiner(
            embed_dim=self.embed_dim,
            pos_dim=self.pos_dim,
            num_classes=num_classes,
            residual_mask=(1, 1, 1, 1, 1, 1, 1, 1)
        )

    # ====== 便捷几何函数 ======
    @staticmethod
    def _blend_yaw(vec_pred: torch.Tensor, vec_gt: torch.Tensor, ratio: float) -> torch.Tensor:
        """单位化向量插值 yaw"""
        v = (1.0 - ratio) * vec_pred + ratio * vec_gt
        return v / (v.norm(dim=-1, keepdim=True).clamp_min(1e-6))

    def _ncd_scalar(self, p: torch.Tensor, t: torch.Tensor) -> torch.Tensor:
        diag = torch.sqrt(t[..., 3] * t[..., 3] + t[..., 4] * t[..., 4] + self.norm_eps)
        h = t[..., 5].clamp_min(self.min_size)

        dx = (p[..., 0] - t[..., 0]).abs() / diag
        dy = (p[..., 1] - t[..., 1]).abs() / diag
        dz = (p[..., 2] - t[..., 2]).abs() / h
        return dx + dy + dz

    def _ncd_vec(self, p: torch.Tensor, t: torch.Tensor) -> torch.Tensor:
        diag = torch.sqrt(t[..., 3] * t[..., 3] + t[..., 4] * t[..., 4] + self.norm_eps)
        h = t[..., 5].clamp_min(self.min_size)

        dx = (p[..., 0] - t[..., 0]) / diag
        dy = (p[..., 1] - t[..., 1]) / diag
        dz = (p[..., 2] - t[..., 2]) / h
        return torch.stack([dx.abs(), dy.abs(), dz.abs()], dim=-1)

    @staticmethod
    def _rel_to_world_cs(box: torch.Tensor) -> torch.Tensor:
        """相对视线角 → 世界角"""
        cx, cy = box[..., 0], box[..., 1]
        cos_r, sin_r = box[..., 6], box[..., 7]
        phi = torch.atan2(cy, cx)
        theta_w = torch.atan2(sin_r, cos_r) + phi
        cos_w, sin_w = torch.cos(theta_w), torch.sin(theta_w)
        return torch.cat([box[..., :6], cos_w.unsqueeze(-1), sin_w.unsqueeze(-1)], dim=-1)

    @staticmethod
    def _world_to_rel_cs(box: torch.Tensor) -> torch.Tensor:
        """世界角 → 相对视线角"""
        cx, cy = box[..., 0], box[..., 1]
        cos_w, sin_w = box[..., 6], box[..., 7]
        phi = torch.atan2(cy, cx)
        theta_rel = torch.atan2(sin_w, cos_w) - phi
        cos_r, sin_r = torch.cos(theta_rel), torch.sin(theta_rel)
        return torch.cat([box[..., :6], cos_r.unsqueeze(-1), sin_r.unsqueeze(-1)], dim=-1)

    # =========================
    #        PREDICT
    # =========================
    def forward_predict(self,
                        feats_vb: torch.Tensor,  # (B, C, Hf, Wf)
                        q: torch.Tensor,  # (B, Q, C)
                        box: torch.Tensor,  # (B, Q, 8) 相对角
                        *,
                        prev_q: torch.Tensor,  # (B, K, C)
                        prev_box: torch.Tensor,  # (B, K, 8)
                        ):
        B, Q, _ = box.shape

        # A)（已移除）地面对齐；直接使用输入 box
        box = box.clone()

        # C) 注意力
        pos = self.box_pe(box)  # (B,Q,P)

        q = self.layernorm1(q)
        if self.use_temporal_attn:
            prev_pos = self.box_pe(prev_box)
            q = self.temporal_attn(cur_q=q, cur_pos=pos, prev_q=prev_q, prev_pos=prev_pos)
        if self.use_self_attn:
            q = self.self_attn(q=q, pos=pos)

        # D) 关键点 & 单视角注入（无 teacher）
        q = self.layernorm2(q)
        pts_pred = self.kpt_gen(q, box)  # (B,Q,K,3)
        q = self.cross_inject(q, pos, pts_pred, feats_vb)

        q = self.layernorm3(q)
        q = self.ffn(q)

        # E) Refine（不再二次地面对齐）
        new_box, cls_logits = self.refiner(q, pos, box)
        return q, new_box, cls_logits

    # =========================
    #          TRAIN
    # =========================
    def forward_train(self,
                      feats_vb: torch.Tensor,
                      q: torch.Tensor,
                      box: torch.Tensor,
                      prev_q: torch.Tensor,
                      prev_box: torch.Tensor,
                      *,
                      teacher_ratio: float = 0.0,
                      target_box: torch.Tensor,
                      target_cls: torch.Tensor,
                      logits_hint: torch.Tensor,  # ← 分类提示仅接受 logits
                      fixed_assign: Optional[Iterable[Tuple[torch.Tensor, torch.Tensor]]] = None  # ← 新增
                      ):
        """仅在 train 模式下调用；包含 teacher 匹配（固定配对）、几何/关键点混合与注入。"""
        assert (target_box is not None) and (target_cls is not None), "train 路径需要 target_box/target_cls"

        B, Q, _ = box.shape

        # A)（已移除）地面对齐；直接使用输入 box
        box = box.clone()

        # B) teacher 几何混合（优先使用固定配对）
        box_for_this = box
        sel_gt_rel = None

        device = box.device
        sel_gt_world = torch.zeros_like(box)
        matched_mask = torch.zeros(B, Q, dtype=torch.bool, device=device)

        with torch.no_grad():
            # —— 使用固定配对 —— #
            for b in range(B):
                idx_pred, idx_tgt_global = fixed_assign[b]
                if idx_pred.numel() == 0:
                    continue
                matched_mask[b, idx_pred] = True
                tb_sel = target_box[b].index_select(0, idx_tgt_global)  # world 角
                sel_gt_world[b].index_copy_(0, idx_pred, tb_sel)

        if matched_mask.any():
            sel_gt_rel = self._world_to_rel_cs(sel_gt_world)
            r = float(teacher_ratio)

            box_mixed = box.clone()
            m = matched_mask.unsqueeze(-1)  # (B, Q, 1)，便于在最后一维广播

            # 0:6 中心+尺寸线性插值
            interp_06 = (1.0 - r) * box[..., :6] + r * sel_gt_rel[..., :6]
            box_mixed[..., :6] = torch.where(m, interp_06, box[..., :6])

            # 6:8 朝向单位化向量插值
            yaw_pred = box[..., 6:8]
            yaw_gt = sel_gt_rel[..., 6:8]
            yaw_mix = self._blend_yaw(yaw_pred, yaw_gt, r)  # 已单位化
            box_mixed[..., 6:8] = torch.where(m, yaw_mix, yaw_pred)

            box_for_this = box_mixed

        # C) 注意力（Temporal → Self）
        pos = self.box_pe(box_for_this)
        q = self.layernorm1(q)
        if self.use_temporal_attn:
            prev_pos = self.box_pe(prev_box)
            q = self.temporal_attn(cur_q=q, cur_pos=pos, prev_q=prev_q, prev_pos=prev_pos)
        if self.use_self_attn:
            q = self.self_attn(q=q, pos=pos)

        # D) 关键点生成 & （可选）teacher 关键点混合（沿用固定配对）
        q = self.layernorm2(q)
        pts_pred = self.kpt_gen(q, box_for_this)  # (B,Q,K,3)

        final_pts = pts_pred
        if (sel_gt_rel is not None) and matched_mask.any():
            with torch.no_grad():
                ncd_val = self._ncd_vec(box_for_this, sel_gt_world).sum(dim=-1)  # (B,Q)
                mask_good = matched_mask & (ncd_val < self.teacher_ncd_thresh)
                if mask_good.any():
                    teacher_pts_full = self.kpt_gen(q.detach(), sel_gt_rel).detach()
                    teacher_pts = torch.where(mask_good[..., None, None], teacher_pts_full, pts_pred)
                    r = float(teacher_ratio)
                    final_pts = pts_pred + r * (teacher_pts - pts_pred)

        # E) 单视角注入
        q, grid = self.cross_inject(q, pos, final_pts, feats_vb)

        q = self.layernorm3(q)
        q = self.ffn(q)

        # F) Refine
        new_box, cls_logits = self.refiner(q, pos, box_for_this)
        return q, new_box, cls_logits, grid

    # 统一入口（无 ground_zmap）
    def forward(self,
                feats_vb: torch.Tensor,  # (B, C, Hf, Wf)
                q: torch.Tensor,  # (B, Q, C)
                box: torch.Tensor,  # (B, Q, 8)
                prev_q: torch.Tensor,
                prev_box: torch.Tensor,
                *,
                teacher_ratio: float = 0.0,
                target_box: Optional[torch.Tensor] = None,
                target_cls: Optional[torch.Tensor] = None,
                logits_hint: Optional[torch.Tensor] = None,  # ← 仅接受 logits
                fixed_assign: Optional[Iterable[Tuple[torch.Tensor, torch.Tensor]]] = None  # ← 新增
                ):

        if self.training:
            return self.forward_train(
                feats_vb, q, box,
                prev_q=prev_q, prev_box=prev_box,
                teacher_ratio=teacher_ratio,
                target_box=target_box,
                target_cls=target_cls,
                logits_hint=logits_hint,
                fixed_assign=fixed_assign
            )
        else:
            return self.forward_predict(
                feats_vb, q, box,
                prev_q=prev_q, prev_box=prev_box,
            )


class DetTaskHead(nn.Module):
    def __init__(self,
                 *,
                 num_queries: int,
                 num_classes: int,
                 embed_dim: int,
                 num_decoder_layers: int = 4,
                 mem_M: int = 40):
        super().__init__()
        self.num_queries = int(num_queries)
        self.num_classes = int(num_classes)
        self.embed_dim = int(embed_dim)
        self.pos_dim = int(embed_dim // 2)
        self.num_layers = int(num_decoder_layers)
        self.M = int(mem_M)

        split = max(1, self.num_layers // 2)
        self.layers = nn.ModuleList([
            DetDecoderLayer(
                embed_dim=self.embed_dim,
                pos_dim=self.pos_dim,
                num_classes=self.num_classes + 1,  # 含背景0
                n_heads=6,
                dropout=0.1,
                num_learnable_pts=6,
                use_temporal_attn=(i < split),
                use_self_attn=(i >= split),
            )
            for i in range(self.num_layers)
        ])
        self.ws = [0.25, 0.25, 0.5, 0.5]

        self.matcher = Hungarian3DMatcher()
        self.criterion = Hungarian3DDetLoss()
        self.criterion_grid = GridLoss()

    @torch.no_grad()
    def _build_fixed_assign(self,
                            prop_box_topk: torch.Tensor,  # [B,Q,8]
                            cls_logits_topk: torch.Tensor,  # [B,Q,C+1]
                            target_box: torch.Tensor,  # [B,GT,8]
                            target_cls: torch.Tensor  # [B,GT]
                            ) -> list[tuple[torch.Tensor, torch.Tensor]]:
        B, Q, _ = prop_box_topk.shape
        pairs: list[tuple[torch.Tensor, torch.Tensor]] = []
        for b in range(B):
            logits_b = cls_logits_topk[b]
            pb = prop_box_topk[b]
            tc_all = target_cls[b]
            tb_all = target_box[b]
            fg_mask = (tc_all >= 1)
            if (Q == 0) or (fg_mask.sum() == 0):
                dev = pb.device
                pairs.append((torch.empty(0, dtype=torch.long, device=dev),
                              torch.empty(0, dtype=torch.long, device=dev)))
                continue

            idx_fg = torch.nonzero(fg_mask, as_tuple=False).flatten()
            tb = tb_all.index_select(0, idx_fg)
            tc = tc_all.index_select(0, idx_fg).to(torch.long)

            cost = self.matcher.build_cost(pred_box=pb, tgt_box=tb, tgt_cls=tc, logits=logits_b)
            r, c = self.matcher.match(cost)
            if r.numel() == 0:
                dev = pb.device
                pairs.append((torch.empty(0, dtype=torch.long, device=dev),
                              torch.empty(0, dtype=torch.long, device=dev)))
            else:
                idx_tgt_global = idx_fg.index_select(0, c)
                pairs.append((r, idx_tgt_global))
        return pairs

    def forward_train(
            self,
            feats_vb: torch.Tensor,
            q0: torch.Tensor,
            box0: torch.Tensor,
            logits0: torch.Tensor,
            prev_q: torch.Tensor,
            prev_box: torch.Tensor,
            *,
            target_det_box: torch.Tensor,
            target_det_cls: torch.Tensor,
            teacher_ratio: float = 0.0
    ):
        fixed_assign = self._build_fixed_assign(
            prop_box_topk=box0,
            cls_logits_topk=logits0,
            target_box=target_det_box,
            target_cls=target_det_cls,
        )

        q = q0
        box = box0
        last_logits = logits0

        zero = feats_vb.sum() * 0.0
        lctr = zero
        lsize = zero
        lyaw = zero
        lcls = zero
        lgrid = zero

        for li in range(self.num_layers):
            q, box, cls_logitsi, grid = self.layers[li](
                feats_vb=feats_vb,
                q=q, box=box,
                prev_q=prev_q, prev_box=prev_box,
                target_box=target_det_box,
                target_cls=target_det_cls,
                logits_hint=last_logits,
                teacher_ratio=float(teacher_ratio),
                fixed_assign=fixed_assign,
            )

            loss_i = self.criterion(
                det_box=box,
                det_cls_logit=cls_logitsi,
                target_box=target_det_box,
                target_cls=target_det_cls,
                fixed_assign=fixed_assign,
            )

            center_mask = torch.zeros(grid.shape[:-1], device=grid.device, dtype=torch.bool)
            center_mask[..., 0] = True
            loss_grid_i = self.criterion_grid(grid=grid, loss_mask=center_mask)

            w = self.ws[li] if li < len(self.ws) else 1.0
            lctr = lctr + w * loss_i["loss_center"]
            lsize = lsize + w * loss_i["loss_size"]
            lyaw = lyaw + w * loss_i["loss_yaw"]
            lcls = lcls + w * loss_i["loss_cls"]
            lgrid = lgrid + w * loss_grid_i

            last_logits = cls_logitsi

        aux = self.criterion(
            det_box=box0,
            det_cls_logit=logits0,
            target_box=target_det_box,
            target_cls=target_det_cls,
            fixed_assign=fixed_assign,
        )
        lsize = lsize + 0.25 * aux["loss_size"]
        lyaw = lyaw + 0.25 * aux["loss_yaw"]
        lcls = lcls + 0.25 * aux["loss_cls"]

        det_box = box
        det_cls = torch.softmax(last_logits, dim=-1)
        det_q = q

        loss = {
            "det_center": lctr,
            "det_size": lsize,
            "det_cls": lcls,
            "det_yaw": lyaw,
            "det_grid": lgrid,
        }
        return det_box, det_cls, det_q, loss

    def forward_predict(
            self,
            feats_vb: torch.Tensor,
            q0: torch.Tensor,
            box0: torch.Tensor,
            logits0: torch.Tensor,
            prev_q: torch.Tensor,
            prev_box: torch.Tensor
    ):
        q = q0
        box = box0
        last_logits = logits0
        for li in range(self.num_layers):
            q, box, cls_logitsi = self.layers[li](
                feats_vb=feats_vb,
                q=q, box=box,
                prev_q=prev_q, prev_box=prev_box,
            )
            last_logits = cls_logitsi

        det_box = box
        det_cls = torch.softmax(last_logits, dim=-1)
        det_q = q
        return det_box, det_cls, det_q

    def forward(
            self,
            feats_vb: torch.Tensor,
            q0: torch.Tensor,
            box0: torch.Tensor,
            logits0: torch.Tensor,
            prev_q: torch.Tensor,
            prev_box: torch.Tensor,
            *,
            target_det_box: Optional[torch.Tensor] = None,
            target_det_cls: Optional[torch.Tensor] = None,
            teacher_ratio: float = 0.0
    ):
        if self.training:
            return self.forward_train(
                feats_vb, q0, box0, logits0, prev_q, prev_box,
                target_det_box=target_det_box,
                target_det_cls=target_det_cls,
                teacher_ratio=teacher_ratio,
            )
        else:
            return self.forward_predict(
                feats_vb, q0, box0, logits0, prev_q, prev_box
            )


class Sparse4DMultiTaskNet(nn.Module):
    """
    单视角，无地面建模：
      - proposal_head 放在 Sparse4DMultiTaskNet 里
      - Sparse4DMultiTaskNet 负责首帧 prev_q/prev_box 的“张量化”初始化（ONNX 友好）
      - DetTaskHead 只做 decoder + loss（输入 seeds + prev）
    """

    def __init__(self,
                 *,
                 enable_det: bool = False,
                 num_det_queries = 100,
                 num_det_cls = 5,
                 enable_lane: bool = True,
                 enable_trj: bool = False,
                 ):
        super().__init__()

        self.enable_lane = bool(enable_lane)
        self.enable_trj = bool(enable_trj)
        self.enable_det = bool(enable_det)

        self.backbone = YOLO11Backbone(scale="x")
        self.embed_dim = int(self.backbone.out_chs[0])
        self.ms_fuse = Conv(sum(self.backbone.out_chs), self.embed_dim, k=1)

        # ===== 任务配置 =====
        self.num_det_queries = num_det_queries
        self.num_det_cls = num_det_cls

        # Det 组件：proposal 在 net，head 仅 decoder
        self.det_proposal: Optional[BoxProposalHead] = None
        self.det_head: Optional[DetTaskHead] = None

        # prev 的“首帧默认值”buffer（ONNX-friendly，无 Python if）
        self.register_buffer("prev_q0", torch.empty(0), persistent=True)  # (1,Q,C)
        self.register_buffer("prev_box0", torch.empty(0), persistent=True)  # (1,Q,8)

        if self.enable_det:
            anchor_path = "/opt_disk2/rd23442/Projects-AD/Dev-AD/train-python/datasets/kmeans_anchors_1000.npy"
            sample_xyz = self._load_anchor_centers(anchor_path=anchor_path)  # (N,3)

            self.det_proposal = BoxProposalHead(
                sample_xyz=sample_xyz,
                topk=self.num_det_queries,
                embed_dim=self.embed_dim,
                pos_dim=self.embed_dim // 2,
                num_classes=self.num_det_cls,
                x_offsets=(-3, 0, 3),
                y_offsets=(-3, 0, 3),
                z_offsets=(-1.0, 1.0),
                n_heads=6,
                dropout=0.1,
                align_corners=True,
                eps=1e-3,
                min_size=1e-3,
            )

            self.det_head = DetTaskHead(
                num_queries=self.num_det_queries,
                num_classes=self.num_det_cls,
                embed_dim=self.embed_dim,
                num_decoder_layers=4,
                mem_M=40,
            )

            # --- 构造 prev_q0 / prev_box0 ---
            Q = int(self.num_det_queries)
            C = int(self.embed_dim)

            prev_q0 = torch.zeros(1, Q, C, dtype=torch.float32)
            self.prev_q0 = prev_q0

            ctr = sample_xyz[:Q].to(torch.float32)  # (Q,3)

            # 这三个值只是“首帧占位”的合理默认；你如果有更合适的先验就替换这里
            size = torch.tensor([4.0, 1.8, 1.6], dtype=torch.float32).view(1, 3).expand(Q, 3)
            cs = torch.tensor([1.0, 0.0], dtype=torch.float32).view(1, 2).expand(Q, 2)

            prev_box0 = torch.cat([ctr, size, cs], dim=-1).unsqueeze(0)  # (1,Q,8)
            self.prev_box0 = prev_box0

        # 其他任务（你原来先不做）
        self.lane_head = None
        self.trj_head = None
        self.trj_loss_fn = None
        self.lane_loss_fn = None

    @staticmethod
    @torch.no_grad()
    def _load_anchor_centers(anchor_path: str) -> torch.Tensor:
        if (anchor_path is None) or (not os.path.isfile(anchor_path)):
            raise FileNotFoundError(f"必须提供有效的 anchor_path，当前为：{anchor_path}")
        try:
            arr = np.load(anchor_path)
            anchors = torch.from_numpy(arr).float()  # (N,8) or (N,>=3)
        except Exception as e:
            raise RuntimeError(f"读取 anchors 失败：{e}")

        if anchors.dim() != 2 or anchors.size(1) < 3:
            raise ValueError(f"anchors 形状必须为 (N,8) 或至少含前三列 (N,>=3)，当前 {tuple(anchors.shape)}")

        centers = anchors[:, 0:3].contiguous()  # (N,3)
        return centers

    def _prep_prev(self,
                   prev_q: torch.Tensor,  # (B,Q,C)
                   prev_box: torch.Tensor,  # (B,Q,8)
                   is_first: torch.Tensor  # (B,) or (B,1)
                   ) -> tuple[torch.Tensor, torch.Tensor]:
        # is_first: 1 表示首帧，用 buffer 替换；0 表示用输入 prev
        m = is_first.reshape(-1, 1, 1).to(dtype=prev_q.dtype, device=prev_q.device)

        q0 = self.prev_q0.to(device=prev_q.device, dtype=prev_q.dtype).expand(prev_q.size(0), -1, -1)
        b0 = self.prev_box0.to(device=prev_box.device, dtype=prev_box.dtype).expand(prev_box.size(0), -1, -1)

        prev_q = m * q0 + (1.0 - m) * prev_q
        prev_box = m * b0 + (1.0 - m) * prev_box
        return prev_q, prev_box

    @torch.no_grad()
    def set_calibration(self, H, W, K, R, t):
        stride = self.backbone.strides[0]

        ref = next(self.parameters())
        device, dtype = ref.device, ref.dtype

        K = torch.as_tensor(K, device=device, dtype=dtype)
        R = torch.as_tensor(R, device=device, dtype=dtype)
        t = torch.as_tensor(t, device=device, dtype=dtype)

        assert K.shape == (3, 3) and R.shape == (3, 3) and t.shape == (3,)

        if (self.det_head is not None) and (self.det_proposal is not None):
            for layer in self.det_head.layers:
                layer.cross_inject.set_calibration(H=H, W=W, stride=stride, K=K, R=R, t=t)
            self.det_proposal.set_calibration(H=H, W=W, stride=stride, K=K, R=R, t=t)

        self.calib_ctx = {"H": H, "W": W, "stride": stride, "K": K, "R": R, "t": t}

    def forward(
        self,
        img: torch.Tensor,
        prev_q_det: torch.Tensor,
        prev_box_det: torch.Tensor,
        *,
        is_first: torch.Tensor,
        target_det_box: Optional[torch.Tensor] = None,
        target_det_cls: Optional[torch.Tensor] = None,
        teacher_ratio: Optional[torch.Tensor] = None
    ):
        p3, p4, p5 = self.backbone(img)
        H3, W3 = int(p3.shape[2]), int(p3.shape[3])

        p4_up = F.interpolate(p4, size=(H3, W3), mode="bilinear", align_corners=True)
        p5_up = F.interpolate(p5, size=(H3, W3), mode="bilinear", align_corners=True)

        fused = torch.cat([p3, p4_up, p5_up], dim=1)
        fused = self.ms_fuse(fused)  # (B, embed_dim, Hf, Wf)

        if not self.enable_det:
            # 你目前 det-only，这里给个明确报错更安全
            raise RuntimeError("enable_det=False but forward() called in det path")

        assert self.det_proposal is not None and self.det_head is not None

        prev_q_det, prev_box_det = self._prep_prev(prev_q_det, prev_box_det, is_first)

        # proposal -> seeds
        q0, box0, logits0 = self.det_proposal(fused)  # (B,Q,C), (B,Q,8), (B,Q,C+1)
        ratio_f = float(teacher_ratio) if teacher_ratio is not None else 0.0

        if self.training:
            det_box, det_cls, det_q, det_loss = self.det_head(
                fused, q0, box0, logits0, prev_q_det, prev_box_det,
                target_det_box=target_det_box,
                target_det_cls=target_det_cls,
                teacher_ratio=ratio_f,
            )
            return det_box, det_cls, det_q, det_loss
        else:
            det_box, det_cls, det_q = self.det_head(
                fused, q0, box0, logits0, prev_q_det, prev_box_det,
            )
            return det_box, det_cls, det_q


class Hungarian3DMatcher:
    """
    3D DETR 风格匈牙利匹配器：
    代价 = w_cls*cost_cls + w_ctr*cost_center + w_size*cost_size + w_yaw*cost_yaw
    - 预测框: yaw 为相对视线角（cos,sin）
    - GT框:   yaw 为世界角（cos,sin），内部转为相对视线角再比对
    - 分类项来源：仅接受 logits（N, C+1），内部用 tau 做 softmax
    """

    def __init__(self,
                 *,
                 w_xyz: Tuple[float, float, float] = (1.0, 1.0, 1.0),
                 w_lwh: Tuple[float, float, float] = (1.0, 1.0, 1.0),
                 cost_cls_w: float = 1.0,
                 cost_center_w: float = 6.0,
                 cost_size_w: float = 0.5,
                 cost_yaw_w: float = 0.2,
                 cls_cost_tau: float = 1.0,
                 norm_eps: float = 1e-3,
                 min_size: float = 1e-2):
        self.w_xyz = torch.as_tensor(w_xyz, dtype=torch.float32)
        self.w_lwh = torch.as_tensor(w_lwh, dtype=torch.float32)
        self.cost_cls_w = float(cost_cls_w)
        self.cost_center_w = float(cost_center_w)
        self.cost_size_w = float(cost_size_w)
        self.cost_yaw_w = float(cost_yaw_w)
        self.cls_cost_tau = float(cls_cost_tau)
        self.norm_eps = float(norm_eps)
        self.min_size = float(min_size)

    @staticmethod
    def _bev_diag(l: torch.Tensor, w: torch.Tensor, eps: float):
        return torch.sqrt(l * l + w * w + eps)

    def _rel_yaw_vec_from_world(self, box_world: torch.Tensor) -> torch.Tensor:
        cx, cy = box_world[..., 0], box_world[..., 1]
        cos_w, sin_w = box_world[..., 6], box_world[..., 7]
        phi = torch.atan2(cy, cx)
        theta_rel = torch.atan2(sin_w, cos_w) - phi
        return torch.stack([torch.cos(theta_rel), torch.sin(theta_rel)], dim=-1)

    def _center_cost(self, P: torch.Tensor, T: torch.Tensor) -> torch.Tensor:
        eps = self.norm_eps
        diag = self._bev_diag(T[:, 3], T[:, 4], eps)  # (K,)
        h = T[:, 5].clamp_min(self.min_size)  # (K,)

        dx = (P[:, 0:1] - T[:, 0]).abs() / diag  # (N,K)
        dy = (P[:, 1:2] - T[:, 1]).abs() / diag  # (N,K)
        dz = (P[:, 2:3] - T[:, 2]).abs() / h  # (N,K)

        w_xyz = self.w_xyz.to(device=P.device, dtype=P.dtype).view(1, 1, 3)
        ctr = torch.stack([dx, dy, dz], dim=-1) * w_xyz
        return ctr.mean(dim=-1)  # (N,K)

    def _size_cost(self, P: torch.Tensor, T: torch.Tensor) -> torch.Tensor:
        # 直接 |Δlwh|
        ps = P[:, 3:6][:, None, :].expand(-1, T.size(0), -1)
        ts = T[:, 3:6][None, :, :].expand(P.size(0), -1, -1)
        w_lwh = self.w_lwh.to(device=P.device, dtype=P.dtype).view(1, 1, 3)
        return (ps.sub(ts).abs() * w_lwh).mean(dim=-1)  # (N,K)

    def _yaw_cost(self, P: torch.Tensor, T: torch.Tensor) -> torch.Tensor:
        p_rel = P[:, 6:8][:, None, :].expand(-1, T.size(0), -1)  # (N,K,2)
        t_rel = self._rel_yaw_vec_from_world(T)[None, :, :].expand(P.size(0), -1, -1)  # (N,K,2)
        dot = (F.normalize(p_rel, dim=-1) * F.normalize(t_rel, dim=-1)).sum(dim=-1).clamp(-1.0, 1.0)
        return 1.0 - dot

    def _cls_cost(self,
                  logits: Optional[torch.Tensor],
                  tgt_cls: torch.Tensor) -> Optional[torch.Tensor]:
        if logits is None:
            return None
        # 内部温度 softmax，防止外部提前归一化带来歧义
        prob = F.softmax(logits / self.cls_cost_tau, dim=-1)  # (N,C+1)
        return -torch.log(prob[:, tgt_cls.to(torch.long)].clamp_min(1e-8))  # (N,K)

    @torch.no_grad()
    def build_cost(self,
                   pred_box: torch.Tensor,  # (N,8)  相对角
                   tgt_box: torch.Tensor,  # (K,8)  世界角
                   tgt_cls: torch.Tensor,  # (K,)
                   *,
                   logits: Optional[torch.Tensor] = None  # (N,C+1)
                   ) -> torch.Tensor:
        assert pred_box.dim() == 2 and pred_box.size(-1) == 8
        assert tgt_box.dim() == 2 and tgt_box.size(-1) == 8
        N, K = pred_box.size(0), tgt_box.size(0)
        if N == 0 or K == 0:
            return pred_box.new_zeros((N, K))

        c_cls = self._cls_cost(logits, tgt_cls)  # (N,K) or None
        c_ctr = self._center_cost(pred_box, tgt_box)  # (N,K)
        c_siz = self._size_cost(pred_box, tgt_box)  # (N,K)
        c_yaw = self._yaw_cost(pred_box, tgt_box)  # (N,K)

        cost = self.cost_center_w * c_ctr + self.cost_size_w * c_siz + self.cost_yaw_w * c_yaw
        if c_cls is not None:
            cost = cost + self.cost_cls_w * c_cls
        return cost

    @staticmethod
    @torch.no_grad()
    def match(cost: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        N, K = cost.shape
        if N == 0 or K == 0:
            dev = cost.device
            return (torch.empty(0, dtype=torch.long, device=dev),
                    torch.empty(0, dtype=torch.long, device=dev))
        r, c = linear_sum_assignment(cost.detach().cpu().to(torch.float64).numpy())
        dev = cost.device
        return (torch.as_tensor(r, dtype=torch.long, device=dev),
                torch.as_tensor(c, dtype=torch.long, device=dev))


# =========================
# 匈牙利 bbox 检测损失（中心 z→地面点 g_z）
# =========================
class Hungarian3DDetLoss(nn.Module):
    """
    3D DETR 风格检测损失（去掉 ground_alpha，尺寸误差不用 log）：
      - 匈牙利匹配：与解码层一致（分类项传 logits）
      - 分类：所有查询 vs 背景/前景（含背景类0，背景权重=eos_coef）
      - 回归：仅匹配到的查询计算
      - 中心项：dx, dy 以 sqrt(l^2+w^2) 归一；dz 直接按高度 h 归一（不再用 cz - α·h）
      - 尺寸项：Δlwh（线性差，不取 log）
      - 朝向项：相对视线角（单位化向量的 1-cos）
    """

    def __init__(self,
                 *,
                 loss_center_w: float = 6.0,
                 loss_size_w: float = 0.5,
                 loss_yaw_w: float = 0.2,
                 robust: str = "huber",  # "huber" | "l1"
                 huber_delta: float = 1.0,
                 norm_eps: float = 1e-3,
                 w_xyz: Tuple[float, float, float] = (1.0, 1.0, 1.0),
                 w_lwh: Tuple[float, float, float] = (1.0, 1.0, 1.0),
                 min_size: float = 1e-2,
                 eos_coef: float = 0.1,
                 label_smoothing: float = 0.0):
        super().__init__()
        assert robust in ("huber", "l1")

        # 加权系数
        self.loss_center_w = float(loss_center_w)
        self.loss_size_w = float(loss_size_w)
        self.loss_yaw_w = float(loss_yaw_w)

        self.robust = robust
        self.huber_delta = float(huber_delta)
        self.norm_eps = float(norm_eps)
        self.min_size = float(min_size)
        self.w_xyz = torch.tensor(w_xyz, dtype=torch.float32)
        self.w_lwh = torch.tensor(w_lwh, dtype=torch.float32)
        self.eos_coef = float(eos_coef)
        self.label_smoothing = float(label_smoothing)

        # 与解码器共用的匹配器（分类项用 logits）
        self.matcher = Hungarian3DMatcher()

    # ==== 内部工具函数 ====

    def _robust_fn(self, x: torch.Tensor):
        if self.robust == "l1":
            return x.abs()
        d = self.huber_delta
        ax = x.abs()
        m = (ax < d).to(x.dtype)
        return 0.5 * (ax ** 2) / d * m + (ax - 0.5 * d) * (1.0 - m)

    def _ncd(self, p: torch.Tensor, t: torch.Tensor):
        # dx, dy：按 bev 对角线归一；dz：直接用高度 h 归一（不再使用 ground_alpha）
        dx = (p[..., 0] - t[..., 0]) / torch.sqrt(t[..., 3] * t[..., 3] + t[..., 4] * t[..., 4] + self.norm_eps)
        dy = (p[..., 1] - t[..., 1]) / torch.sqrt(t[..., 3] * t[..., 3] + t[..., 4] * t[..., 4] + self.norm_eps)
        dz = (p[..., 2] - t[..., 2]) / (t[..., 5].clamp_min(self.min_size))
        return torch.stack([dx, dy, dz], dim=-1)

    def _size_res(self, p: torch.Tensor, t: torch.Tensor):
        # 直接 Δlwh（线性差），不取 log
        return p[..., 3:6] - t[..., 3:6]

    # ==== 前向 ====
    def forward(self,
                det_box: torch.Tensor,  # [B, D, 8]   预测框（相对视线角）
                det_cls_logit: torch.Tensor,  # [B, D, C+1] 分类 logits（含背景0）
                target_box: torch.Tensor,  # [B, K, 8]   GT（世界角）
                target_cls: torch.Tensor,  # [B, K]
                *,
                fixed_assign: Optional[Iterable[Tuple[torch.Tensor, torch.Tensor]]] = None
                ):
        device = det_box.device
        dtype = det_box.dtype
        B, D, _ = det_box.shape
        C_plus_1 = det_cls_logit.size(-1)
        bg_id = 0

        empty_weight = torch.ones(C_plus_1, device=device, dtype=dtype)
        empty_weight[bg_id] = self.eos_coef

        w_xyz = self.w_xyz.to(device=device, dtype=dtype)
        w_lwh = self.w_lwh.to(device=device, dtype=dtype)

        total_center = det_box.new_tensor(0.0)
        total_size = det_box.new_tensor(0.0)
        total_yaw = det_box.new_tensor(0.0)
        total_cls = det_box.new_tensor(0.0)

        num_targets = 0
        num_matched = 0

        use_fixed = fixed_assign is not None

        for b in range(B):
            pb = det_box[b]  # [D,8]
            logits_b = det_cls_logit[b]  # [D,C+1]
            cls_b = target_cls[b]  # [K]
            labels = torch.full((D,), bg_id, dtype=torch.long, device=device)

            # 统计前景 GT 数（用于分类归一化）
            fg_mask = (cls_b >= 1)
            K_eff = int(fg_mask.sum().item())
            num_targets += K_eff

            if use_fixed:
                idx_pred, idx_tgt_global = fixed_assign[b]
                M = int(idx_pred.numel())
                num_matched += M

                if M == 0:
                    # 只有分类：全部当成背景
                    total_cls = total_cls + F.cross_entropy(
                        logits_b, labels, weight=empty_weight,
                        label_smoothing=self.label_smoothing, reduction="sum"
                    )
                    continue

                # 分类标签：按固定配对写入前景类
                labels[idx_pred] = cls_b.index_select(0, idx_tgt_global).to(torch.long)

                # 分类损失（所有查询 vs 背景/前景）
                total_cls = total_cls + F.cross_entropy(
                    logits_b, labels, weight=empty_weight,
                    label_smoothing=self.label_smoothing, reduction="sum"
                )

                # 回归损失：仅匹配对
                pbm = pb.index_select(0, idx_pred)  # [M,8]
                tbm = target_box[b].index_select(0, idx_tgt_global)  # [M,8]

                # center: 归一化 dx,dy,dz
                ncd_vec = self._robust_fn(self._ncd(pbm, tbm)) * w_xyz
                total_center = total_center + ncd_vec.sum()

                # size: Δlwh
                s_res = self._robust_fn(self._size_res(pbm, tbm)) * w_lwh
                total_size = total_size + s_res.sum()

                # yaw: 相对角向量相似度（1 - cos）
                theta_w = torch.atan2(tbm[:, 7], tbm[:, 6])  # world yaw
                phi = torch.atan2(tbm[:, 1], tbm[:, 0])  # 视线方位
                theta_rel = theta_w - phi
                t_rel = torch.stack([torch.cos(theta_rel), torch.sin(theta_rel)], dim=-1)  # [M,2]
                p_rel = pbm[:, 6:8]
                yaw_l = 1.0 - (F.normalize(p_rel, dim=-1) * F.normalize(t_rel, dim=-1)).sum(dim=-1).clamp(-1.0, 1.0)
                total_yaw = total_yaw + yaw_l.sum()
                continue

            # ===== 兼容旧路径：内部重匹配 =====
            idx_fg = torch.nonzero(fg_mask, as_tuple=False).flatten()
            if idx_fg.numel() == 0:
                total_cls = total_cls + F.cross_entropy(
                    logits_b, labels, weight=empty_weight,
                    label_smoothing=self.label_smoothing, reduction="sum"
                )
                continue

            tb = target_box[b, idx_fg]
            tc = cls_b[index_fg].to(torch.long) if (index_fg := idx_fg) is not None else cls_b[idx_fg].to(
                torch.long)  # 兼容性写法

            cost = self.matcher.build_cost(
                pred_box=pb, tgt_box=tb, tgt_cls=tc, logits=logits_b
            )
            idx_pred, idx_tgt = self.matcher.match(cost)
            M = int(idx_pred.numel())
            num_matched += M
            if M > 0:
                labels[idx_pred] = tc.index_select(0, idx_tgt)

            total_cls = total_cls + F.cross_entropy(
                logits_b, labels, weight=empty_weight,
                label_smoothing=self.label_smoothing, reduction="sum"
            )

            if M > 0:
                pbm = pb.index_select(0, idx_pred)
                tbm = tb.index_select(0, idx_tgt)

                ncd_vec = self._robust_fn(self._ncd(pbm, tbm)) * w_xyz
                total_center = total_center + ncd_vec.sum()

                s_res = self._robust_fn(self._size_res(pbm, tbm)) * w_lwh
                total_size = total_size + s_res.sum()

                theta_w = torch.atan2(tbm[:, 7], tbm[:, 6])
                phi = torch.atan2(tbm[:, 1], tbm[:, 0])
                theta_rel = theta_w - phi
                t_rel = torch.stack([torch.cos(theta_rel), torch.sin(theta_rel)], dim=-1)
                p_rel = pbm[:, 6:8]
                yaw_l = 1.0 - (F.normalize(p_rel, dim=-1) * F.normalize(t_rel, dim=-1)).sum(dim=-1).clamp(-1.0, 1.0)
                total_yaw = total_yaw + yaw_l.sum()

        norm_cls = max(int(num_targets), 1)
        norm_reg = max(int(num_matched), 1)

        loss_center = self.loss_center_w * (total_center / norm_reg)
        loss_size = self.loss_size_w * (total_size / norm_reg)
        loss_yaw = self.loss_yaw_w * (total_yaw / norm_reg)
        loss_cls = total_cls / norm_cls

        return {
            'loss_center': loss_center,
            'loss_size': loss_size,
            'loss_yaw': loss_yaw,
            'loss_cls': loss_cls,
        }


# =========================
# 自检 + ONNX 导出（det-only, 2D Ground, 单视角）
# =========================
if __name__ == "__main__":
    import onnx
    from onnx import shape_inference
    import onnxruntime as ort

    torch.manual_seed(0)
    np.random.seed(0)

    model = Sparse4DMultiTaskNet(enable_lane=False, enable_trj=False, enable_det=True).eval()

    H, W, B = 256, 512, 1
    eye3 = torch.eye(3, dtype=torch.float32)
    zero3 = torch.zeros(3, dtype=torch.float32)

    # 世界(x前,y左,z上) → 相机(x右,y下,z前)
    R_cw = torch.tensor([
        [0.0, -1.0, 0.0],  # x_c
        [0.0, 0.0, -1.0],  # y_c
        [1.0, 0.0, 0.0],  # z_c
    ], dtype=torch.float32)

    model.set_calibration(
        H=H, W=W,
        K=eye3, R=R_cw, t=zero3,
    )

    img = torch.randn(B, 3, H, W)

    d_embed = int(model.embed_dim)
    Dq = getattr(model, "num_det_queries", 0)
    is_first = torch.ones(B, 1)  # 首帧就 1；非首帧就 0
    prev_q_det = torch.randn(B, Dq, d_embed)
    prev_box_det = torch.ones(B, Dq, 8)  # [cx,cy,cz,l,w,h,cos,sin]


    class DetExportWrapper(nn.Module):
        def __init__(self, net: nn.Module):
            super().__init__()
            self.net = net

        def forward(self, img, prev_q_det, prev_box_det, is_first):
            det_box, det_cls, det_q = self.net(
                img,
                prev_q_det=prev_q_det,
                prev_box_det=prev_box_det,
                is_first=is_first,
            )
            return det_box, det_cls, det_q


    wrapper = DetExportWrapper(model).eval()

    tmp_raw = "sparse4d_det_tmp_raw.onnx"
    final_onnx = "sparse4d_det.onnx"
    os.makedirs(os.path.dirname(tmp_raw) or ".", exist_ok=True)

    input_names = ["img", "prev_q_det", "prev_box_det", "is_first"]
    output_names = ['det_box', 'det_cls', 'q_det']
    torch.onnx.export(
        wrapper,
        (img, prev_q_det, prev_box_det, is_first),
        tmp_raw,
        export_params=True,
        opset_version=17,
        do_constant_folding=False,
        input_names=input_names,
        output_names=output_names,
    )
    print(f"[OK] Exported ONNX (raw tmp): {tmp_raw}")

    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_BASIC
    so.optimized_model_filepath = final_onnx
    _ = ort.InferenceSession(tmp_raw, so, providers=["CPUExecutionProvider"])
    print(f"[OK] ORT BASIC graph optimized and saved: {final_onnx}")

    m = onnx.load(final_onnx)
    m = shape_inference.infer_shapes(m)
    onnx.save(m, final_onnx)
    print(f"[OK] Shapes inferred and saved in-place: {final_onnx}")

    try:
        os.remove(tmp_raw)
        print(f"[CLEAN] Removed temp: {tmp_raw}")
    except OSError:
        pass

    print(f"[DONE] Final det-only model: {final_onnx}")
