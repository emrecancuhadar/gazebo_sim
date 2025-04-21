#!/usr/bin/env python3
import cv2
import numpy as np
import json

def main():
    # Update this path to your actual forest image.
    img_path = '/home/emrecan/two_wheel_ws/src/gazebo_sim/ros_gz_example_description/models/my_ground_plane/materials/textures/forest.png'
    img = cv2.imread(img_path)
    if img is None:
        print(f"ERROR: Could not load image at {img_path}")
        return

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    thresh_value = 65
    _, mask_dark = cv2.threshold(gray, thresh_value, 255, cv2.THRESH_BINARY)

    grid_rows, grid_cols = 10, 10
    height, width = gray.shape
    cell_height = height // grid_rows
    cell_width = width // grid_cols

    dark_percentages = np.zeros((grid_rows, grid_cols))
    for i in range(grid_rows):
        for j in range(grid_cols):
            start_y = i * cell_height
            end_y = (i + 1) * cell_height if i < grid_rows - 1 else height
            start_x = j * cell_width
            end_x = (j + 1) * cell_width if j < grid_cols - 1 else width

            cell_mask = mask_dark[start_y:end_y, start_x:end_x]
            dark_count = np.count_nonzero(cell_mask == 0)
            total_pixels = cell_mask.size
            dark_percentage = (dark_count / total_pixels) * 100
            dark_percentages[i, j] = dark_percentage

    # Map [0..100]% to intensities (not used further in this pseudo setup)
    intensities = 0.015 * dark_percentages + 0.5
    print(json.dumps(intensities.tolist()))

if __name__ == '__main__':
    main()
