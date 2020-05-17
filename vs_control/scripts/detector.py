import cv2
import numpy as np

def adjust_bbox(bbox, x_limit=1280, y_limit=800):
    x1, y1, w, h = bbox
    x2 = x1 + w
    y2 = y1 + h
    if x1 < 0:
        x1 = 0
    elif x1 > x_limit - 1:
        x1 = x_limit - 1
    if x2 < 0:
        x2 = 0
    elif x2 > x_limit - 1:
        x2 = x_limit - 1
    if y1 < 0:
        y1 = 0
    elif y1 > y_limit - 1:
        y1 = y_limit - 1
    if y2 < 0:
        y2 = 0
    elif y2 > y_limit - 1:
        y2 = y_limit - 1
    w = x2 - x1
    h = y2 - y1
    return (x1, y1, w, h)

class MarkerDetector:
    def __init__(self):
        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
        self.marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    def search(self, img):
        found = False
        corners, ids, _ = cv2.aruco.detectMarkers(img, self.marker_dict, parameters=self.detector_params)
        if ids is not None and 23 in ids:
            found = True
            index = np.where(ids == 23)[0][0]
            corners = corners[index]
        return found, corners

class LogoDetector:
    def __init__(self):
        self.marker_detector = MarkerDetector()
    def search(self, img):
        found, corners = self.marker_detector.search(img)
        if found == False:
            return False, ()
        H, _ = cv2.findHomography(np.float32([[[420, -480]], [[780, -480]], [[780, -120]], [[420, -120]]]), corners)
        cnt = cv2.perspectiveTransform(np.float32([[[0, 0]], [[1200, 0]], [[1200, 600]], [[0, 600]]]), H)
        bbox = cv2.boundingRect(cnt)
        x, y, w, h = adjust_bbox(bbox)
        if w * h == 0:
            return False, ()
        return True, (x, y, w, h)