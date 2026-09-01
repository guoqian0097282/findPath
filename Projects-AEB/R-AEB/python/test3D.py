import cv2
import numpy as np

# ============================================================
# 数据字符串
# ============================================================
data_str = """1 480.122 259.425 486.577 261.577 441.394 263.728 439.242 261.577 480.122 218.545 486.577 218.545 441.394 216.394 439.242 218.545
2 584.756 267.97 625.636 265.819 716.002 272.274 672.97 276.577 584.756 222.787 625.636 222.787 716.002 218.484 672.97 218.484
2 548.545 257.274 572.213 257.274 613.092 261.577 578.667 263.728 548.545 225 572.213 225 613.092 222.848 578.667 222.848
0 846.455 265.88 842.151 265.88 855.061 263.728 859.364 265.88 846.455 207.787 842.151 207.787 855.061 207.787 859.364 207.787
"""

def parse_3d_box(line):
    """
    解析一行数据，返回 label 和 8个点
    点顺序: 0,1,2,3 是底部点，4,5,6,7 是顶部点
    """
    parts = line.strip().split()
    if len(parts) < 17:
        return None, None
    
    label = int(float(parts[0]))
    points = []
    
    for i in range(1, len(parts), 2):
        if i + 1 < len(parts):
            x = float(parts[i])
            y = float(parts[i + 1])
            points.append((x, y))
    
    return label, points

def draw_3d_box_on_image(image, data_str, color_by_label=True):
    """
    在读取的图像上绘制3D框
    """
    img_copy = image.copy()
    
    colors = {
        0: (0, 255, 0),    # 绿色
        1: (0, 0, 255),    # 红色
        2: (255, 0, 0),    # 蓝色
        3: (0, 255, 255),  # 黄色
        4: (255, 0, 255),  # 粉色
    }
    
    lines = data_str.strip().split('\n')
    
    for line_idx, line in enumerate(lines):
        if not line.strip():
            continue
            
        label, points = parse_3d_box(line)
        if points is None or len(points) < 8:
            continue
        
        # 分离底部点和顶部点
        bottom_points = np.array(points[:4], dtype=np.int32)  # 点 0,1,2,3
        top_points = np.array(points[4:8], dtype=np.int32)    # 点 4,5,6,7
        
        # 选择颜色
        if color_by_label:
            color = colors.get(label, (255, 255, 255))
        else:
            color = (0, 255, 0)
        
        # 绘制底部矩形（点0,1,2,3）
        cv2.polylines(img_copy, [bottom_points], isClosed=True, color=color, thickness=2)
        
        # 绘制顶部矩形（点4,5,6,7）
        cv2.polylines(img_copy, [top_points], isClosed=True, color=color, thickness=2)
        
        # 绘制垂直连接线（0-4, 1-5, 2-6, 3-7）
        for i in range(4):
            pt1 = tuple(bottom_points[i])
            pt2 = tuple(top_points[i])
            cv2.line(img_copy, pt1, pt2, color, 2)
        
        # 绘制底部点（红色实心圆）
        for i, (x, y) in enumerate(bottom_points):
            cv2.circle(img_copy, (int(x), int(y)), 5, (0, 0, 255), -1)
            cv2.putText(img_copy, str(i), (int(x)+6, int(y)-6), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        
        # 绘制顶部点（蓝色实心圆）
        for i, (x, y) in enumerate(top_points):
            idx = i + 4
            cv2.circle(img_copy, (int(x), int(y)), 5, (255, 0, 0), -1)
            cv2.putText(img_copy, str(idx), (int(x)+6, int(y)-6), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
        
        # 标注 label
        centroid = np.mean(bottom_points, axis=0).astype(np.int32)
        cv2.putText(img_copy, f"Label: {label}", 
                   (centroid[0]-30, centroid[1]-30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    
    return img_copy

# ============================================================
# 主程序：读取图像并绘制
# ============================================================
if __name__ == "__main__":
    # 读取图像（替换为您的图像路径）
    image_path = "/home/gq/guoqian/Projects-AEB/raeb_vis_ori_0.jpg"  # 修改为您的图像路径
    image = cv2.imread(image_path)
    
    # 如果图像读取失败，创建空白图像
    if image is None:
        print(f"Warning: Cannot read {image_path}, creating blank image")
        image = np.ones((600, 800, 3), dtype=np.uint8) * 255
    
    # 在图像上绘制3D框
    result = draw_3d_box_on_image(image, data_str)
    
    # 显示图像
    cv2.imshow("3D Boxes on Image", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # 保存结果
    cv2.imwrite("result_with_boxes.png", result)
    print("Saved to result_with_boxes.png")