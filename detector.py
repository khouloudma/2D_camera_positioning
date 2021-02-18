import cv2
import numpy as np


def detect(image):
    x1 = 0
    y1 = 0
    x2 = 0
    y2 = 0
    x3 = 0
    y3 = 0
    # param_du_detecteur
    params = cv2.SimpleBlobDetector_Params()
    params.filterByCircularity = True
    params.minCircularity = 0.8
    params.filterByArea = True
    params.minArea = 3000
    params.maxArea = 30000

    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(image)

    (a,) = np.shape(keypoints)
    if a >= 1:
        x1 = keypoints[0].pt[0]

        y1 = keypoints[0].pt[1]
    if a >= 2:
        x1 = keypoints[0].pt[0]
        y1 = keypoints[0].pt[1]
        x2 = keypoints[1].pt[0]
        y2 = keypoints[1].pt[1]

    if a >= 3:
        x1 = keypoints[0].pt[0]
        y1 = keypoints[0].pt[1]
        x2 = keypoints[1].pt[0]
        y2 = keypoints[1].pt[1]
        x3 = keypoints[2].pt[0]
        y3 = keypoints[2].pt[1]

    out = cv2.drawKeypoints(image, keypoints, np.array([0]), (0, 0, 255),
                            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    return out, x1, y1, x2, y2, x3, y3
