#!/usr/bin/env python3
import numpy as np
import rasterio
import sys
import json, time, os, subprocess
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors  # For creating a custom colormap

def read_band(file_path):
    """
    Reads a raster band and returns the data as a float32 array scaled to 0-1.
    Assumes the input reflectance is scaled (e.g., 0-10000) and divides accordingly.
    """
    with rasterio.open(file_path) as src:
        data = src.read(1).astype('float32')
        profile = src.profile
    # Scale from 0-10000 to 0-1
    data /= 10000.0
    return data

def compute_ndvi(nir, red):
    """
    Computes NDVI = (NIR - Red) / (NIR + Red) and avoids division by zero.
    """
    return (nir - red) / (nir + red + 1e-6)

def grid_majority_class(classification, num_rows=10, num_cols=10):
    """
    Downscales the classification to a coarse grid by assigning each grid cell
    the majority class found within it.
    """
    height, width = classification.shape
    coarse = np.zeros((num_rows, num_cols), dtype=np.uint8)
    
    row_size = height // num_rows
    col_size = width // num_cols
    
    for r in range(num_rows):
        for c in range(num_cols):
            row_start = r * row_size
            row_end   = height if r == num_rows - 1 else (r + 1) * row_size
            col_start = c * col_size
            col_end   = width if c == num_cols - 1 else (c + 1) * col_size
            
            sub_arr = classification[row_start:row_end, col_start:col_end]
            values, counts = np.unique(sub_arr, return_counts=True)
            
            coarse[r, c] = values[np.argmax(counts)]
    return coarse

def classify_seasonal(summer_ndvi, winter_ndvi):
    """
    Classifies pixels using summer and winter NDVI.
    
    Logic:
      - If summer NDVI < 0.45, classify as Non-vegetation / Bare Soil (class 1).
      - If summer NDVI is between 0.45 and 0.65, classify as Sparse Vegetation (class 4).
      - For pixels with summer NDVI >= 0.65 (dense vegetation/forest):
          Compute amplitude = summer_ndvi - winter_ndvi.
          If amplitude > 0.2, classify as Deciduous Forest (class 2),
          otherwise classify as Coniferous Forest (class 3).
          
    Returns a classification array with:
       1: Non-vegetation / Bare Soil
       2: Deciduous Forest
       3: Coniferous Forest
       4: Sparse Vegetation
    """
    cls = np.zeros(summer_ndvi.shape, dtype=np.uint8)
    
    # Non-vegetation / Bare Soil: Very low summer NDVI
    nonveg = summer_ndvi < 0.5
    cls[nonveg] = 1
    
    # Sparse Vegetation: Moderate summer NDVI but not dense
    sparse = (summer_ndvi >= 0.5) & (summer_ndvi < 0.65)
    cls[sparse] = 4
    
    # Dense vegetation (forest) mask: summer NDVI >= 0.65
    forest = summer_ndvi >= 0.65
    # Calculate the seasonal difference (amplitude)
    amplitude = summer_ndvi - winter_ndvi
    # Among forest pixels, if amplitude > 0.2: Deciduous; else Coniferous.
    deciduous = forest & (amplitude > 0.2)
    coniferous = forest & (amplitude <= 0.2)
    cls[deciduous] = 2
    cls[coniferous] = 3
    
    return cls

if __name__ == '__main__':
    # expects 4 args: summer_red, summer_nir, winter_red, winter_nir
    if len(sys.argv) != 5:
        print(json.dumps([])); sys.exit(0)
    sr_fp, sn_fp, wr_fp, wn_fp = sys.argv[1:]
    try:
        sr = read_band(sr_fp)
        sn = read_band(sn_fp)
        wr = read_band(wr_fp)
        wn = read_band(wn_fp)
        ndvi_s = compute_ndvi(sn, sr)
        ndvi_w = compute_ndvi(wn, wr)
        cls = classify_seasonal(ndvi_w, ndvi_s)
        summary = grid_majority_class(cls)
        print(json.dumps(summary.tolist()))
    except Exception as e:
        print(e)
        print(json.dumps([]))