#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
批量重命名图片脚本
功能：读取文件夹中所有图片，从 0 开始重命名，并替换原文件
支持格式：jpg, jpeg, png, bmp, webp, tiff
"""

import os
import sys
from pathlib import Path

# ============ 支持的图片格式 ============
IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.bmp', '.webp', '.tiff', '.tif'}


def get_image_files(folder_path):
    """获取文件夹中所有图片文件（按文件名排序）"""
    image_files = []
    for file in Path(folder_path).iterdir():
        if file.is_file() and file.suffix.lower() in IMAGE_EXTENSIONS:
            image_files.append(file)
    
    # 按文件名排序，确保顺序一致
    image_files.sort(key=lambda x: x.name)
    return image_files


def rename_images(folder_path, start_index=0, dry_run=False):
    """
    批量重命名图片
    
    Args:
        folder_path: 文件夹路径
        start_index: 起始编号（默认 0）
        dry_run: True=只预览不执行，False=执行重命名
    """
    image_files = get_image_files(folder_path)
    
    if not image_files:
        print(f"❌ 文件夹 '{folder_path}' 中没有找到图片文件")
        return
    
    print(f"📁 找到 {len(image_files)} 张图片")
    print("=" * 60)
    
    # 计算需要填充的位数
    total_digits = len(str(len(image_files) - 1 + start_index))
    
    # 显示预览
    print("📋 预览重命名结果：")
    for idx, file_path in enumerate(image_files):
        new_name = f"{idx + start_index:0{total_digits}d}{file_path.suffix}"
        print(f"   {file_path.name} -> {new_name}")
    
    print("=" * 60)
    
    if dry_run:
        print("🔍 预览模式结束（未执行任何更改）")
        return
    
    # 确认执行
    confirm = input("⚠️  确认执行重命名？(y/n): ").strip().lower()
    if confirm != 'y':
        print("❌ 已取消操作")
        return
    
    # 执行重命名
    renamed_count = 0
    error_count = 0
    
    for idx, file_path in enumerate(image_files):
        new_name = f"{idx + start_index:0{total_digits}d}{file_path.suffix}"
        new_path = file_path.parent / new_name
        
        try:
            # 如果目标文件已存在，先处理
            if new_path.exists():
                print(f"⚠️  目标文件已存在: {new_name}，跳过")
                error_count += 1
                continue
            
            file_path.rename(new_path)
            renamed_count += 1
            print(f"✅ {file_path.name} -> {new_name}")
        except Exception as e:
            print(f"❌ 重命名失败: {file_path.name} -> {e}")
            error_count += 1
    
    print("=" * 60)
    print(f"✅ 成功重命名 {renamed_count} 张图片")
    if error_count > 0:
        print(f"⚠️  失败 {error_count} 张")


def main():
    # ============ 配置区 ============
    # 方式1: 直接在这里设置文件夹路径
    folder_path = input("请输入图片文件夹路径: ").strip()
    
    # 如果直接回车，使用当前目录
    if not folder_path:
        folder_path = "."
    
    # 起始编号（默认 0）
    start_input = input("请输入起始编号（默认 0）: ").strip()
    start_index = int(start_input) if start_input else 0
    
    # 是否预览
    dry_input = input("是否仅预览？(y/n, 默认 n): ").strip().lower()
    dry_run = dry_input == 'y'
    # =================================
    
    if not os.path.exists(folder_path):
        print(f"❌ 文件夹不存在: {folder_path}")
        sys.exit(1)
    
    rename_images(folder_path, start_index, dry_run)


if __name__ == "__main__":
    main()