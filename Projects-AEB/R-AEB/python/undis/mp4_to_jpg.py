import cv2
import os

def extract_frames_to_jpg(video_path, output_dir, frame_interval=1, start_frame=0, max_frames=None):
    """
    从MP4视频中提取帧并保存为JPG格式
    
    Args:
        video_path: 视频文件路径
        output_dir: 输出目录
        frame_interval: 帧间隔（1表示每帧都保存，2表示隔一帧保存）
        start_frame: 起始帧索引
        max_frames: 最大提取帧数，None表示全部提取
    """
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    # 打开视频
    cap = cv2.VideoCapture(video_path)
    
    if not cap.isOpened():
        print(f"Error: Cannot open video file {video_path}")
        return
    
    # 获取视频信息
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    print(f"Video Info:")
    print(f"  Total frames: {total_frames}")
    print(f"  FPS: {fps}")
    print(f"  Resolution: {width}x{height}")
    print(f"  Output directory: {output_dir}")
    
    # 设置起始帧
    if start_frame > 0:
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
    
    frame_count = 0
    saved_count = 0
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            break
        
        # 检查是否达到最大帧数
        if max_frames is not None and saved_count >= max_frames:
            break
        
        # 根据帧间隔决定是否保存
        if frame_count % frame_interval == 0:
            # 生成文件名（使用6位数字补齐）
            filename = f"frame_{saved_count:06d}.jpg"
            filepath = os.path.join(output_dir, filename)
            
            # 保存为JPG（质量参数 90）
            cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
            saved_count += 1
            
            # 进度显示
            if saved_count % 100 == 0:
                print(f"  Saved {saved_count} frames...")
        
        frame_count += 1
    
    # 释放资源
    cap.release()
    
    print(f"Extraction complete!")
    print(f"  Total frames processed: {frame_count}")
    print(f"  Frames saved: {saved_count}")
    print(f"  Output directory: {output_dir}")

# ============================================================
# 使用示例
# ============================================================
if __name__ == "__main__":
    # 基本使用
    # extract_frames_to_jpg(
    #     video_path="/home/gq/guoqian/Projects-AEB/M112/1/VID_20251231_230000_GEN_1.mp4",
    #     output_dir="./images",
    #     frame_interval=1,   # 每帧都保存
    #     start_frame=0,      # 从第0帧开始
    #     max_frames=None     # 保存所有帧
    # )
    
    # 示例：每隔10帧保存一帧
    extract_frames_to_jpg(
        video_path="/home/gq/guoqian/Projects-AEB/M112/1/VID_20251231_230000_GEN_1.mp4",
        output_dir="./images",
        frame_interval=3
    )