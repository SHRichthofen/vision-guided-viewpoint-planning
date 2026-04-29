# utils.py
import math

def get_centered_crop_coords(bbox_xywh, img_w, img_h, padding_ratio=0.2):
    """
    计算居中的裁剪坐标
    bbox_xywh: (center_x, center_y, width, height)
    """
    cx, cy, w, h = bbox_xywh
    
    pad_w = w * padding_ratio
    pad_h = h * padding_ratio
    
    new_w = w + 2 * pad_w
    new_h = h + 2 * pad_h
    
    x1 = int(cx - new_w / 2)
    y1 = int(cy - new_h / 2)
    x2 = int(cx + new_w / 2)
    y2 = int(cy + new_h / 2)
    
    # 边界限制 (Clamp)
    x1 = max(0, x1)
    y1 = max(0, y1)
    x2 = min(img_w, x2)
    y2 = min(img_h, y2)
    
    return x1, y1, x2, y2

def project_point_3d_to_2d(p3d, intr):
    """
    将 3D 点投影回 2D 像素坐标
    p3d: [x, y, z] (米)
    intr: RealSense 内参对象
    """
    if p3d[2] <= 0: 
        return (0, 0)
    
    u = int(intr.fx * p3d[0] / p3d[2] + intr.ppx)
    v = int(intr.fy * p3d[1] / p3d[2] + intr.ppy)
    return (u, v)
