#!/usr/bin/env python3
import cv2
import numpy as np
import json
import sys

def compute_intensity(img_path: str, thresh=75, rows=10, cols=10) -> list[list[float]]:
    img = cv2.imread(img_path)
    if img is None:
        raise FileNotFoundError(f"Image not found: {img_path}")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)
    h, w = gray.shape
    ch, cw = h // rows, w // cols
    grid = []
    for i in range(rows):
        row = []
        y0, y1 = i*ch, h if i == rows-1 else (i+1)*ch
        for j in range(cols):
            x0, x1 = j*cw, w if j == cols-1 else (j+1)*cw
            sub = mask[y0:y1, x0:x1]
            dark = int((sub == 0).sum())
            pct = (dark / sub.size) * 100
            row.append(0.015 * pct + 0.5)
        grid.append(row)
    return grid

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(json.dumps([])); sys.exit(0)
    try:
        grid = compute_intensity(sys.argv[1])
        print(json.dumps(grid))
    except:
        print(json.dumps([]))