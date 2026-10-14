#!/usr/bin/env python3
import cv2
import cv2.aruco as aruco
import numpy as np
import os

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
marker_id = 0
marker_size = 160   # 从 400 缩小到 160

marker_img = aruco.drawMarker(dictionary, marker_id, marker_size)

margin = 40         # 从 80 缩小到 40
canvas_size = marker_size + 2 * margin
canvas = np.ones((canvas_size, canvas_size), dtype=np.uint8) * 255
canvas[margin:margin+marker_size, margin:margin+marker_size] = marker_img

out_dir = os.path.expanduser('~/robotics_study/experiments')
os.makedirs(out_dir, exist_ok=True)
out_path = os.path.join(out_dir, 'aruco_marker_0.png')
cv2.imwrite(out_path, canvas)

print(f'标记已保存: {out_path}')
print(f'尺寸: {canvas_size} x {canvas_size} 像素')
