import cv2
import os
import sys

def h264_to_jpg_opencv(video_path, output_dir, extract_interval=1, max_frames=None):
    """
    使用 OpenCV 将 H.264 视频提取为 JPG 图片
    
    Args:
        video_path: 视频文件路径
        output_dir: 输出目录
        extract_interval: 提取间隔（每隔多少帧提取一张）
        max_frames: 最大提取帧数
    """
    # 创建输出目录
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # 打开视频
    cap = cv2.VideoCapture(video_path)
    
    if not cap.isOpened():
        print(f"Error: Cannot open video file: {video_path}")
        return
    
    # 获取视频信息
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    print(f"Video Info:")
    print(f"  FPS: {fps}")
    print(f"  Total frames: {total_frames}")
    print(f"  Output dir: {output_dir}")
    print(f"  Extract interval: {extract_interval}")
    print("-" * 50)
    
    frame_count = 0
    saved_count = 0
    # 使用 5 位数字作为文件名编号
    num_digits = len(str(total_frames))
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 按间隔提取
        if frame_count % extract_interval == 0:
            # 生成文件名
            filename = f"frame_{str(frame_count).zfill(num_digits)}.jpg"
            filepath = os.path.join(output_dir, filename)
            
            # 保存为 JPG
            cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            saved_count += 1
            
            # 进度显示
            if saved_count % 10 == 0:
                print(f"  Saved {saved_count} images... (frame {frame_count})")
            
            # 检查最大帧数
            if max_frames and saved_count >= max_frames:
                print(f"Reached max frames limit: {max_frames}")
                break
        
        frame_count += 1
    
    cap.release()
    print(f"Completed! Saved {saved_count} images from {frame_count} frames.")

# 使用示例
if __name__ == "__main__":
    # 方式1：基本使用
    # h264_to_jpg_opencv(
    #     video_path="/home/gq/guoqian/Projects-AEB/VID_20251231_230000_GEN_2.h264",
    #     output_dir="./images2",
    #     extract_interval=30,  # 每30帧保存一张（大约每秒1张，30fps）
    #     max_frames=100  # 最多保存100张
    # )
    
    # 方式2：提取所有帧
    h264_to_jpg_opencv(
        video_path="/home/gq/guoqian/Projects-AEB/M112/260817/General_rear/VID_20260101_000417_GEN_2.h264",
        output_dir="./images_rear",
        extract_interval=10
    )