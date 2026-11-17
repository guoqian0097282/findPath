# extract_video_from_mf4.py
"""
从 MF4 文件中提取 / 解码视频帧，保存为 JPG 图片。

用法:
    python extract_video_from_mf4.py input.mf4 output_dir
"""
import os
import sys
import numpy as np
import cv2
from asammdf import MDF


# ---------------- 基础工具 ----------------

def to_bytes(x) -> bytes | None:
    """把 MF4 样本统一转成 bytes"""
    if isinstance(x, (bytes, bytearray)):
        return bytes(x)
    if isinstance(x, np.ndarray):
        # 常见: uint8 数组
        try:
            return x.astype(np.uint8).tobytes()
        except Exception:
            return None
    if isinstance(x, (np.void, memoryview)):
        return bytes(x)
    return None


def is_jpeg(b: bytes) -> bool:
    return len(b) >= 2 and b[0] == 0xFF and b[1] == 0xD8


def is_png(b: bytes) -> bool:
    return len(b) >= 4 and b[:4] == b"\x89PNG"


def is_mp4(b: bytes) -> bool:
    return len(b) >= 12 and b[4:8] == b"ftyp"


# ---------------- 1. 从 Attachment 提取 ----------------

def extract_from_attachments(mdf: MDF, out_dir: str) -> int:
    n = 0
    for att in getattr(mdf, "attachments", []) or []:
        raw = att.extract()
        if isinstance(raw, np.ndarray):
            raw = raw.astype(np.uint8).tobytes()
        elif isinstance(raw, (bytearray, memoryview)):
            raw = bytes(raw)
        if not isinstance(raw, (bytes,)) or not raw:
            continue

        mime = (getattr(att, "mime_type", "") or "").lower()

        if is_jpeg(raw) or mime.startswith("image/jpeg"):
            fn = os.path.join(out_dir, f"att_{n:06d}.jpg")
            with open(fn, "wb") as f:
                f.write(raw)
            n += 1

        elif is_mp4(raw) or mime.startswith("video/"):
            # 存成 mp4 再交给 3 步拆帧
            fn = os.path.join(out_dir, f"att_video_{n:06d}.mp4")
            with open(fn, "wb") as f:
                f.write(raw)
            print(f"[attachment] 保存视频容器: {fn}")
            n += 1

        elif is_png(raw):
            fn = os.path.join(out_dir, f"att_{n:06d}.png")
            with open(fn, "wb") as f:
                f.write(raw)
            n += 1
    return n


# ---------------- 2. 从 Channel 逐帧提取 JPEG ----------------

def _looks_like_video_channel(ch) -> bool:
    name = (getattr(ch, "name", "") or "").lower()
    comment = (getattr(ch, "comment", "") or "").lower()
    kw = ("video", "jpeg", "jpg", "image", "frame", "pic", "camera", "cam")
    return any(k in name for k in kw) or any(k in comment for k in kw)


def extract_jpeg_from_channels(mdf: MDF, out_dir: str) -> int:
    n = 0
    # raw=True 会返回原始样本，避免 asammdf 做单位转换
    for g_idx, group in enumerate(mdf.groups):
        for c_idx, ch in enumerate(group.channels):
            # 优先处理明显是视频的 channel，但也不排斥其它
            # （有些工具命名不规范，所以不全跳过）
            try:
                sig = mdf.get(group=g_idx, index=c_idx, raw=True)
            except Exception:
                continue

            arr = sig.samples
            if arr is None or arr.dtype.kind not in ("O", "S", "V"):
                continue

            # 命中数统计，用于结束提示
            hit = 0
            for elem in arr:
                b = to_bytes(elem)
                if not b or not is_jpeg(b):
                    continue
                fn = os.path.join(out_dir, f"ch_frame_{n:06d}.jpg")
                with open(fn, "wb") as f:
                    f.write(b)
                n += 1
                hit += 1

            if hit:
                tag = "✓" if _looks_like_video_channel(ch) else "·"
                print(f"  {tag} group[{g_idx}]/{ch.name}: 提取 {hit} 帧")
    return n


# ---------------- 3. 把 mp4/h264 视频拆成 jpg ----------------

def mp4_to_jpg(mp4_path: str, out_dir: str, prefix: str, jpeg_quality: int = 90) -> int:
    cap = cv2.VideoCapture(mp4_path)
    if not cap.isOpened():
        print(f"[warn] 打不开视频: {mp4_path}，可能需要 ffmpeg/av")
        return 0
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        fn = os.path.join(out_dir, f"{prefix}_{n:06d}.jpg")
        cv2.imwrite(fn, frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        n += 1
    cap.release()
    return n


# ---------------- 主流程 ----------------

def main(mf4_path: str, out_dir: str):
    os.makedirs(out_dir, exist_ok=True)
    mdf = MDF(mf4_path)
    print(f"打开 {mf4_path}: {len(mdf.groups)} 个 group, "
          f"{len(getattr(mdf, 'attachments', []) or [])} 个 attachment")

    # 1) attachments
    n_att = extract_from_attachments(mdf, out_dir)
    print(f"[附件] 提取 {n_att} 个文件")

    # 2) channel 里的逐帧 JPEG
    print("[Channel 扫描] 查找 JPEG 帧 ...")
    n_ch = extract_jpeg_from_channels(mdf, out_dir)
    print(f"[Channel] 共提取 {n_ch} 帧 JPEG")

    # 3) 把附件里保存的 mp4 拆帧
    n_mp4 = 0
    for fn in os.listdir(out_dir):
        if fn.startswith("att_video_") and fn.endswith(".mp4"):
            n = mp4_to_jpg(os.path.join(out_dir, fn),
                           out_dir,
                           prefix=fn[:-4] + "_frame")
            print(f"[mp4] {fn} → {n} 帧")
            n_mp4 += n

    print(f"\n完成：JPEG 帧 {n_ch} + mp4 拆帧 {n_mp4}，输出目录: {out_dir}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("用法: python extract_video_from_mf4.py input.mf4 output_dir")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])