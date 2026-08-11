#pragma once
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>

struct RaebPostprocResult {
    cv::Mat objs;  // [N,7] CV_32F
    cv::Mat masks; // [N,H,W] CV_8U, may be empty
};

/**
 * @file postproc_api.h
 * @brief RAEB 2D AngleSeg 后处理 C++ API（与 Python 版本对齐）
 *
 * Python 对应文件/函数：
 *   - postproc_InitRAEB(...)
 *   - postproc_ProcRAEB(det_cat, proto, angle) -> objs
 *   - postproc_VisInstances(img, objs, masks, track_info=None) -> vis_img
 *
 * ----------------------------
 * 1) 全局实例与调用顺序
 * ----------------------------
 * - 本模块内部持有一个全局后处理实例（等价 Python 的 g_segpost_raeb）。
 * - 使用前必须先调用 postproc_InitRAEB(...) 完成初始化。
 *
 * ----------------------------
 * 2) 坐标/尺度约定（非常重要）
 * ----------------------------
 * - 本后处理器不做“去 pad / 除 ratio / 映射回原图”的逻辑。
 * - postprocess 输出的 objs 坐标 (x1,y1,x2,y2) 与输出 masks 的坐标系必须一致。
 * - 具体一致性约定与 Python 完全相同：
 *     你需要保证 det_cat 里的 bbox 坐标尺度与输出 masks 的尺度一致，
 *     即 bbox 坐标应落在 mask 输出尺寸 (H0,W0) 的像素坐标系中。
 *
 * ----------------------------
 * 3) 输入数据格式（对齐 Python）
 * ----------------------------
 * A) det_cat：
 *   - 类型：cv::Mat，dims==2，type==CV_32F
 *   - 形状：[C, L]
 *       C = 4 + nc + nm
 *       L = 候选数（anchors / queries / proposals 数量）
 *   - 通道布局（按列对应每个候选）：
 *       det_cat[0:4,  i]   -> bbox (cx, cy, w, h)
 *       det_cat[4:4+nc, i] -> 分类分数（nc 类）
 *       det_cat[4+nc:4+nc+nm, i] -> mask coeff（nm 维）
 *
 * B) proto：
 *   - 类型：cv::Mat，dims==3，type==CV_32F
 *   - 形状：支持两种布局（与 Python 相同）
 *       1) [nm, Hm, Wm]
 *       2) [Hm, Wm, nm]
 *   - Hm/Wm 是原型掩膜的分辨率（低分辨率）。
 *
 * C) angle：
 *   - 类型：cv::Mat，type==CV_32F
 *   - 语义：原始角度张量（由 angle_postproc 配置决定解码方式）
 *   - 允许形状：
 *       1) 直接角度向量：dims==1 [L] / dims==2 [L,1] / [1,L]
 *       2) bins+residual：如 [2K,L] 或 [L,2K]（K 由 angle_bins 决定）
 *
 * ----------------------------
 * 4) 输出数据格式（对齐 Python）
 * ----------------------------
 * A) objs：
 *   - 类型：cv::Mat，dims==2，type==CV_32F
 *   - 形状：[N, 7]
 *   - 每行一个目标：
 *       [x1, y1, x2, y2, conf, cls, theta]
 *     - x1,y1,x2,y2：xyxy 像素坐标（与 masks 坐标系一致）
 *     - conf：分类置信度（该目标的最大类分）
 *     - cls：类别 id（与 id2name 的 key 对应）
 *     - theta：角度（弧度），绘制箭头时遵循 Python 的方向约定：
 *         dx = -L * sin(theta)
 *         dy = -L * cos(theta)
 *       因此 theta=0 指向“上”，theta=pi/2 指向“左”
 *
 * B) masks：
 *   - 类型：cv::Mat，dims==3，type==CV_8U
 *   - 形状：[N, H0, W0]
 *     其中：
 *       H0 = Hm * mask_up
 *       W0 = Wm * mask_up
 *   - 值域：二值 {0,1}
 *   - 语义：
 *       - masks[n, :, :] 是第 n 个目标的掩膜
 *       - 已按对应 bbox 做裁剪：bbox 外区域保证为 0
 *
 * ----------------------------
 * 5) 初始化参数语义（对齐 Python）
 * ----------------------------
 * - nc：类别数量（仅用于快速 sanity check；真正 nc 也可从 det_cat 的 C 推出）
 * - nm：mask 系数维度（proto 通道数）
 * - id2name：类别映射（Python: dict[int,str]）
 *     - JSON 约定：key 为可转 int 的字符串，value 为类别名字符串
 *       例如：{"0":"person","1":"car"}
 * - whitelist：白名单（Python: dict[int,dict]，仅使用 key 集合）
 *     - JSON 约定：key 为可转 int 的字符串，value 内容不关心
 *       例如：{"0":{}, "1":{}}
 * - conf_thresh：分类阈值（> 才保留）
 * - iou_thresh：NMS IoU 阈值（类无关）
 * - mask_thresh：mask 二值化阈值（> 才为 1）
 * - mask_up：输出 mask 缩放倍数（H0/W0 的定义见上）
 */

/**
 * @brief 初始化 RAEB AngleSeg 后处理（对应 Python: postproc_InitRAEB）
 *
 * @param nc          类别数（默认 80）
 * @param nm          mask coeff / proto 通道数（默认 32）
 * @param id2name     类别 id->name 映射（JSON object；key 为 "0","1"...；value 为 string）
 * @param whitelist   白名单（JSON object；key 为 "0","1"...；value 任意 object；仅使用 key）
 * @param conf_thresh 分类阈值（默认 0.5）
 * @param iou_thresh  NMS IoU 阈值（默认 0.3）
 * @param mask_thresh mask 二值化阈值（默认 0.5）
 * @param mask_up     mask 上采样倍数（默认 4）
 * @param angle_postproc 角度后处理配置（JSON object，可为空）
 */
void postproc_InitRAEB(
    int nc,
    int nm,
    const nlohmann::json::object_t& id2name,
    const nlohmann::json::object_t& whitelist,
    float conf_thresh = 0.5f,
    float iou_thresh  = 0.3f,
    float mask_thresh = 0.5f,
    int mask_up = 4,
    const nlohmann::json::object_t& angle_postproc = nlohmann::json::object_t{}
);


/**
 * @brief 单帧后处理：AngleSeg（对应 Python: postproc_ProcRAEB）
 *
 * 输入/输出格式详见文件头注释（非常重要）。
 *
 * @param det_cat   [C,L] CV_32F；C = 4 + nc + nm；bbox = cxcywh；cls_scores；mask_coeff
 * @param proto     3D CV_32F；当 proto_chw=true 时为 [nm,Hm,Wm]；当 proto_chw=false 时为 [Hm,Wm,nm]
 * @param angle     原始角度张量；CV_32F。可为直接角度向量，或 bins+residual 形式
 * @param proto_chw 是否按 [nm,Hm,Wm] 解释 proto：
 *                  - true  : proto 形状为 [nm,Hm,Wm]（NCHW）
 *                  - false : proto 形状为 [Hm,Wm,nm]（HWC）
 *
 * @return objs
 *   - objs : [N,7] CV_32F -> [x1,y1,x2,y2,conf,cls,theta]
 */
cv::Mat postproc_ProcRAEB(
    const cv::Mat& det_cat,
    const cv::Mat& proto,
    const cv::Mat& angle,
    bool proto_chw = true
);

RaebPostprocResult postproc_ProcRAEBWithMasks(
    const cv::Mat& det_cat,
    const cv::Mat& proto,
    const cv::Mat& angle,
    bool proto_chw = true
);

/**
 * @brief 可视化实例结果（不带 tracking；对应 Python: postproc_VisInstances(img, objs, masks)）
 *
 * @param img   (H,W,3) CV_8UC3
 * @param objs  [N,7]  CV_32F
 * @param masks [N,H,W] CV_8U 或可被内部转换的二值 mask。要求 H/W 与 img 一致
 *
 * @return vis_img (H,W,3) CV_8UC3
 *
 * 说明：
 * - 该函数内部会调用底层 AngleSegPostProcessor 的 draw_angleins：
 *   先叠加 mask + bbox + label（复用 vis.hpp），再画角度箭头。
 */
cv::Mat postproc_VisInstances(
    const cv::Mat& img,
    const cv::Mat& objs,
    const cv::Mat& masks
);

/**
 * @brief 可视化实例结果（带 tracking；对应 Python: postproc_VisInstances(img, objs, masks, track_info)）
 *
 * @param img        (H,W,3) CV_8UC3
 * @param objs       [Nt,6] 或 [Nt,7] CV_32F
 *                   - [x1,y1,x2,y2,conf,cls] 或 [x1,y1,x2,y2,conf,cls,theta]
 *                   - 若只有 6 列，则内部按 Python 逻辑补 theta=0
 * @param masks      [Nt,H,W] CV_8U (0/1)，与 objs 同顺序
 * @param track_info [Nt,4] CV_32F -> [track_id, track_state, track_age, idx]
 *
 * @return vis_img (H,W,3) CV_8UC3
 *
 * 文本格式对齐 Python：
 *   "ID <track_id> <class_name> <conf> A<track_age>"
 */
cv::Mat postproc_VisInstances(
    const cv::Mat& img,
    const cv::Mat& objs,
    const cv::Mat& masks,
    const cv::Mat& track_info
);
