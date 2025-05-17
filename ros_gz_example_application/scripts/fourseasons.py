#!/usr/bin/env python3
import numpy as np
import rasterio
import sys
import json
from scipy import ndimage


def read_band(file_path):
    with rasterio.open(file_path) as src:
        data = src.read(1).astype('float32')
    return data / 10000.0


def compute_ndvi(nir, red):
    return (nir - red) / (nir + red + 1e-6)


def grid_majority_classification(cls, num_rows=10, num_cols=10):
    h, w = cls.shape
    row_size, col_size = h // num_rows, w // num_cols
    grid = np.zeros((num_rows, num_cols), dtype=np.uint8)
    for i in range(num_rows):
        for j in range(num_cols):
            rs, re = i*row_size, h if i==num_rows-1 else (i+1)*row_size
            cs, ce = j*col_size, w if j==num_cols-1 else (j+1)*col_size
            block = cls[rs:re, cs:ce]
            vals, cnts = np.unique(block, return_counts=True)
            grid[i,j] = vals[np.argmax(cnts)]
    return grid


def block_stat(arr, num=10, func=np.nanmean):
    h, w = arr.shape
    bh, bw = h//num, w//num
    grid = np.zeros((num, num), dtype=float)
    for i in range(num):
        for j in range(num):
            rs, re = i*bh, h if i==num-1 else (i+1)*bh
            cs, ce = j*bw, w if j==num-1 else (j+1)*bw
            grid[i,j] = func(arr[rs:re, cs:ce])
    return grid


def classify_seasonal(s_ndvi, w_ndvi):
    cls = np.zeros(s_ndvi.shape, dtype=np.uint8)
    cls[s_ndvi < 0.5] = 1
    mask4 = (s_ndvi >= 0.5) & (s_ndvi < 0.65)
    cls[mask4] = 4
    forest = s_ndvi >= 0.65
    amp = s_ndvi - w_ndvi
    cls[forest & (amp > 0.2)]  = 2
    cls[forest & (amp <= 0.2)] = 3
    return cls


if __name__ == '__main__':
    if len(sys.argv) != 5:
        print(json.dumps({}))
        sys.exit(0)
    sr_fp, sn_fp, wr_fp, wn_fp = sys.argv[1:]
    try:
        sr = read_band(sr_fp)
        sn = read_band(sn_fp)
        wr = read_band(wr_fp)
        wn = read_band(wn_fp)
        ndvi_s = compute_ndvi(sn, sr)
        ndvi_w = compute_ndvi(wn, wr)

        cls_full = classify_seasonal(ndvi_s, ndvi_w)
        cls_Grid = grid_majority_classification(cls_full)

        # DEM -> slope & aspect
        dem_path = "/home/emrecan/two_wheel_ws/src/gazebo_sim/ros_gz_example_description/models/my_ground_plane/materials/textures/bizimalandem3.tif"  # or explicit DEM path
        with rasterio.open(dem_path) as src:
            dem = src.read(1).astype('float32')
            tr = src.transform
        xres, yres = tr.a, -tr.e
        dzdx = ndimage.sobel(dem, axis=1)/(8*xres)
        dzdy = ndimage.sobel(dem, axis=0)/(8*yres)
        slope  = np.degrees(np.arctan(np.hypot(dzdx, dzdy)))
        aspect = (np.degrees(np.arctan2(dzdy, -dzdx)) + 360) % 360

        dem_Grid    = block_stat(dem)
        slope_Grid  = block_stat(slope)
        aspect_Grid = block_stat(aspect)

        out = {
            'classification': cls_Grid.tolist(),
            'elevation':      dem_Grid.tolist(),
            'slope':          slope_Grid.tolist(),
            'aspect':         aspect_Grid.tolist()
        }
        print(json.dumps(out))
    except Exception as e:
        print(json.dumps({}))
        sys.exit(1)