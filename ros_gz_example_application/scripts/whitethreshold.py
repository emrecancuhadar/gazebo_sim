#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from cv_bridge import CvBridge
import cv2
import numpy as np

CIRCULARITY_THRESHOLD = 0.6  # 1 ≡ perfect circle

class WhiteMaskDetector:
    def __init__(self, min_area=1):
        self.min_area    = min_area
        self.lower_white = np.array([0, 0, 200])
        self.upper_white = np.array([180, 70, 255])

    def detect(self, frame):
        hsv        = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask       = cv2.inRange(hsv, self.lower_white, self.upper_white)
        blurred    = cv2.medianBlur(mask, 3)
        kernel     = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        sure_bg    = cv2.dilate(blurred, kernel, iterations=1)
        dist       = cv2.distanceTransform(blurred, cv2.DIST_L2, 5)
        _, sure_fg = cv2.threshold(dist, 2.5, 255, cv2.THRESH_BINARY)
        sure_fg    = sure_fg.astype(np.uint8)

        unknown    = cv2.subtract(sure_bg, sure_fg)
        _, markers = cv2.connectedComponents(sure_fg)
        markers   += 1
        markers[unknown == 255] = 0

        markers    = cv2.watershed(frame, markers)
        segmentation = np.zeros_like(blurred)
        segmentation[markers > 1] = 255

        return blurred, sure_bg, sure_fg, segmentation


def is_circular(cnt, thr=CIRCULARITY_THRESHOLD):
    peri = cv2.arcLength(cnt, True)
    if peri == 0:
        return False
    area = cv2.contourArea(cnt)
    circ = 4.0 * np.pi * area / (peri * peri)
    return circ >= thr


class WhiteGridNode(Node):
    def __init__(self):
        super().__init__('white_grid_node')
        self.bridge   = CvBridge()
        self.detector = WhiteMaskDetector(min_area=1)

        # ---- grid setup -------------------------------------------------
        self.grid_rows         = 10
        self.grid_cols         = 10
        self.max_area_per_cell = 45
        self.window_size       = 10
        self.grid_buffer       = []
        self.ball_weight       = 9  # fixed weight per detected contour

        # ---- fisheye calibration ----------------------------------------
        self.K = np.array([
            [510.82238711,   0.0,           293.24081725],
            [0.0,            511.17749131,  242.22831403],
            [0.0,              0.0,           1.0       ]
        ], dtype=np.float64)
        self.D = np.array([
            -0.15209115,
             0.8733624,
            -4.94754539,
             7.71077147
        ], dtype=np.float64)
        # will hold remap once we know image size:
        self._map1 = None
        self._map2 = None

        # ---- ROS I/O ----------------------------------------------------
        self.subscription = self.create_subscription(
            CompressedImage, '/image/compressed', self.image_callback, 10)
        self.grid_pub     = self.create_publisher(
            Float32MultiArray, '/grid/detected_balls', 10)
        self.get_logger().info(
            'Subscribed to /image/compressed; publishing /grid/fire_count')

        # ---- GUI --------------------------------------------------------
        cv2.startWindowThread()
        cv2.namedWindow('Stages',   cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Combined', cv2.WINDOW_AUTOSIZE)

    def image_callback(self, msg):
        # Acquire raw frame
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        if frame.ndim == 2 or (frame.ndim == 3 and frame.shape[2] == 1):
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # ---- undistort fisheye ----------------------------------------
        h, w = frame.shape[:2]
        if self._map1 is None:
            # initialize undistort map once per resolution
            self._map1, self._map2 = cv2.fisheye.initUndistortRectifyMap(
                self.K, self.D, np.eye(3), self.K, (w, h), cv2.CV_16SC2)
        frame = cv2.remap(frame, self._map1, self._map2,
                          interpolation=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT)

        # ---- detection stages ------------------------------------------
        blurred, sure_bg, sure_fg, segmented = self.detector.detect(frame)

        # ---- contour filtering & circular mask -------------------------
        contours, _ = cv2.findContours(
            sure_fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        frame_draw = frame.copy()
        circ_mask  = np.zeros_like(segmented)
        valid      = []

        for cnt in contours:
            if cv2.contourArea(cnt) < self.detector.min_area:
                continue
            if not is_circular(cnt):
                continue

            valid.append(cnt)
            cv2.drawContours(frame_draw, [cnt], -1, (0, 255, 0), 1)
            cv2.drawContours(circ_mask,   [cnt], -1, 255, cv2.FILLED)

        # ---- grid math -------------------------------------------------
        step_y     = h // self.grid_rows
        step_x     = w // self.grid_cols
        # build per-cell masks
        cell_masks = []
        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                m = np.zeros((h, w), dtype=np.uint8)
                y0, y1 = r * step_y, (r + 1) * step_y
                x0, x1 = c * step_x, (c + 1) * step_x
                m[y0:y1, x0:x1] = 255
                cell_masks.append(m)

        cell_areas = np.zeros((self.grid_rows, self.grid_cols), dtype=float)
        for cnt in valid:
            fill = np.zeros((h, w), dtype=np.uint8)
            cv2.drawContours(fill, [cnt], -1, 255, cv2.FILLED)
            overlaps = [
                cv2.countNonZero(cv2.bitwise_and(fill, cm))
                for cm in cell_masks
            ]
            r, c = divmod(int(np.argmax(overlaps)), self.grid_cols)
            cell_areas[r, c] += self.ball_weight

        # ---- smoothing & normalization ---------------------------------
        frac = np.clip(cell_areas / self.max_area_per_cell, 0.0, 1.0)
        self.grid_buffer.append(frac)
        if len(self.grid_buffer) > self.window_size:
            self.grid_buffer.pop(0)
        mean_frac = np.mean(self.grid_buffer, axis=0)

        # ---- publish ----------------------------------------------------
        grid_msg = Float32MultiArray()
        grid_msg.layout = MultiArrayLayout(dim=[
            MultiArrayDimension(label='rows', size=self.grid_rows, stride=self.grid_rows*self.grid_cols),
            MultiArrayDimension(label='cols', size=self.grid_cols, stride=self.grid_cols)
        ])
        grid_msg.data = mean_frac.astype(np.float32).flatten().tolist()
        self.grid_pub.publish(grid_msg)

        # ---- visualization: circular mask + grid -----------------------
        kernel     = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        circ_mask  = cv2.dilate(circ_mask, kernel, iterations=2)
        circ_vis   = cv2.cvtColor(circ_mask, cv2.COLOR_GRAY2BGR)

        for i in range(1, self.grid_rows):
            y = i * step_y
            cv2.line(circ_vis, (0, y), (w, y), (100,100,100), 1)
        for j in range(1, self.grid_cols):
            x = j * step_x
            cv2.line(circ_vis, (x, 0), (x, h), (100,100,100), 1)

        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                val = mean_frac[r, c] * 10.0
                cv2.putText(
                    circ_vis,
                    f'{val:.1f}',
                    (c * step_x + step_x//4, r * step_y + step_y//2),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0,255,255),
                    1
                )

        combined = np.hstack([frame_draw, circ_vis])
        stages   = np.hstack([
            cv2.cvtColor(blurred, cv2.COLOR_GRAY2BGR),
            cv2.cvtColor(sure_bg, cv2.COLOR_GRAY2BGR),
            cv2.cvtColor(sure_fg, cv2.COLOR_GRAY2BGR)
        ])
        cv2.imshow('Stages',   stages)
        cv2.imshow('Combined', combined)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = WhiteGridNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
