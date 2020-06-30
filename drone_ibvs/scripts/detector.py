import cv2
import numpy as np
from threading import Lock, Thread

import rospy
import rospkg

DETECTOR_ERRNO = {"DETE_NOT_FOUND": 1,
                  "INIT_SIFT_INSUF": 2,
                  "INIT_MATCH_INSUF": 3,
                  "INIT_H_REJECT": 4,
                  "INIT_FLOW_INSU": 5,
                  "TRAC_FLOW_INSUF": 6,
                  "TRAC_H_REJECT": 7}

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

def draw_pts(img, pts):
    pts = np.reshape(pts, (-1, 2))
    for pt in pts:
        x = int(pt[0])
        y = int(pt[1])
        cv2.circle(img, (x, y), 3, (0, 255, 0), -1)
    return img

class Detector:
    def search(self, img):
        return -1, None

class TagDetector:
    def __init__(self):
        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
        self.marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    def search(self, img):
        corners, ids, _ = cv2.aruco.detectMarkers(img, self.marker_dict, parameters=self.detector_params)
        if ids is not None and 23 in ids:
            index = np.where(ids == 23)[0][0]
            corners = corners[index]
            return 0, corners
        return DETECTOR_ERRNO["DETE_NOT_FOUND"], None

class LogoDetector:
    def __init__(self, nfeatures=80, min_nfeatures=40):
        self.status = 0
        self.curr_pts = None
        self.prev_pts = None
        self.curr_co_pts = None
        self.prev_co_pts = None
        self.curr_img = None
        self.prev_img = None
        self.H = None
        self.mtx = Lock()
        self.nfeatures = nfeatures
        self.min_nfeatures = min_nfeatures
        self.tag_detector = TagDetector()
        self.f2d_extractor = cv2.xfeatures2d_SIFT.create()
        self.fd_matcher = cv2.BFMatcher()
        self.template_corners = np.float32([[[0, 0]], [[1200, 0]], [[1200, 600]], [[0, 600]]])

        rospack = rospkg.RosPack()
        with open('{}/config/map/kp'.format(rospack.get_path("drone_ibvs")), 'r') as f:
            self.kp0 = np.load(f)
        with open('{}/config/map/des'.format(rospack.get_path("drone_ibvs")), 'r') as f:
            self.des0 = np.load(f)
        
        self.update_thread = None
        self.update_shutdown = True
    def reset(self):
        self.shutdown_update()
        self.status = 0
        self.curr_pts = None
        self.prev_pts = None
        self.curr_co_pts = None
        self.prev_co_pts = None
        self.curr_img = None
        self.prev_img = None
        self.H = None
        self.update_shutdown = True
        return
    def shutdown_update(self):
        if self.update_thread is not None:
            self.update_shutdown = True                
            self.update_thread.join()
            self.update_thread = None
        return
    def start_update(self):
        self.shutdown_update()
        self.update_shutdown = False
        self.update_thread = Thread(target=self.update, args=[1, self.kp0, self.des0, 10])
        self.update_thread.start()
        return
    def search(self, src):
        if self.status == 0:
            found, bbox = self.detect(src)
            if not found:
                return DETECTOR_ERRNO["DETE_NOT_FOUND"], None
            errno = self.initialize(src, bbox, self.kp0, self.des0, min_inliers=10)
            if errno != 0:
                return errno, None
            self.status = 1
            corners = cv2.perspectiveTransform(np.float32([[[420, 120]], [[780, 120]], [[780, 480]], [[420, 480]]]), self.H)
            self.start_update()
            return 0, corners.reshape(4, 2)
        elif self.status == 1:
            errno = self.track(src)
            if errno != 0:
                self.status = 0
                self.shutdown_update()
                return errno, None
            corners = cv2.perspectiveTransform(np.float32([[[420, 120]], [[780, 120]], [[780, 480]], [[420, 480]]]), self.H)
            return 0, corners.reshape(4, 2)
        
    def detect(self, src):
        errno, corners = self.tag_detector.search(src)
        if errno != 0:
            return False, ()
        H, _ = cv2.findHomography(np.float32([[[420, -480]], [[780, -480]], [[780, -120]], [[420, -120]]]), corners)
        cnt = cv2.perspectiveTransform(self.template_corners, H)
        bbox = cv2.boundingRect(cnt)
        x, y, w, h = adjust_bbox(bbox)
        if w * h == 0:
            return False, ()
        return True, (x, y, w, h)
    def initialize(self, src, bbox, kp0, des0, min_inliers=10):
        x, y, w, h = bbox
        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        crop = gray[int(y):int(y + h), int(x):int(x + w)]
        kp, des = self.f2d_extractor.detectAndCompute(crop, mask=None)
        # print(len(kp))
        if kp is None or len(kp) < min_inliers:
            return DETECTOR_ERRNO["INIT_SIFT_INSUF"]
        kp = self.kp2array(kp, x_offset=x, y_offset=y)
        pts0, pts = self.match(kp0, kp, des0, des)
        if pts is None or len(pts) < min_inliers:
            return DETECTOR_ERRNO["INIT_MATCH_INSUF"]
        H, pts0, pts = self.reject_with_H(pts0, pts)
        # print(len(pts))
        if pts is None or len(pts) < min_inliers:
            return DETECTOR_ERRNO["INIT_H_REJECT"]
        cnt = cv2.perspectiveTransform(self.template_corners, H)
        mask = np.zeros_like(gray)
        mask = cv2.fillPoly(mask, [np.reshape(np.int32(cnt), (-1, 2))], 255)
        pts = cv2.goodFeaturesToTrack(gray, self.nfeatures, 0.1, 3, mask=mask)
        if pts is None or len(pts) < self.min_nfeatures:
            return DETECTOR_ERRNO["INIT_FLOW_INSUF"]
        pts0 = cv2.perspectiveTransform(pts, np.linalg.inv(H))
        self.curr_pts = pts
        self.prev_pts = pts
        self.curr_co_pts = pts0
        self.prev_co_pts = pts0
        self.curr_img = gray
        self.prev_img = gray
        self.H = H
        return 0
    def track(self, src):
        self.mtx.acquire()
        self.prev_img = self.curr_img
        self.prev_pts = self.curr_pts
        self.prev_co_pts = self.curr_co_pts
        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        self.curr_img = gray
        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_img, self.curr_img, self.prev_pts, None, winSize=(19, 19),
                                                                            maxLevel=2,
                                                                            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        curr_pts = np.reshape(curr_pts[status==1], (-1, 1, 2))
        if curr_pts is None or len(curr_pts) < self.min_nfeatures:
            self.mtx.release()
            return DETECTOR_ERRNO["TRAC_FLOW_INSUF"]
        curr_co_pts = np.reshape(self.prev_co_pts[status==1], (-1, 1, 2))
        H, curr_co_pts, curr_pts = self.reject_with_H(curr_co_pts, curr_pts)
        curr_co_pts = cv2.perspectiveTransform(curr_pts, np.linalg.inv(H))
        if curr_pts is None or len(curr_pts) < self.min_nfeatures:
            self.mtx.release()
            return DETECTOR_ERRNO["TRAC_H_REJECT"]
        curr_pts_num = curr_pts.shape[0]
        if curr_pts_num < self.nfeatures:
            cnt = cv2.perspectiveTransform(self.template_corners, H)
            mask = np.zeros_like(gray)
            mask = cv2.fillPoly(mask, [np.reshape(np.int32(cnt), (-1, 2))], 255)
            for i in range(curr_pts_num):
                x = int(curr_pts[i][0][0])
                y = int(curr_pts[i][0][1])
                mask = cv2.circle(mask, (x, y), 20, 0, -1)
            add_pts = cv2.goodFeaturesToTrack(gray, self.nfeatures - curr_pts.shape[0], 0.1, 3, mask=mask)
            if add_pts is not None and len(add_pts) > 0:
                add_co_pts = cv2.perspectiveTransform(add_pts, np.linalg.inv(H))
                curr_pts = np.append(curr_pts, add_pts, axis=0)
                curr_co_pts = np.append(curr_co_pts, add_co_pts, axis=0)
        self.curr_pts = curr_pts
        self.curr_co_pts = curr_co_pts
        self.H = H
        self.mtx.release()
        return 0
    def update(self, Hz, kp0, des0, min_inliers=10):
        r = rospy.Rate(Hz)
        while True:
            r.sleep()
            if self.update_shutdown:
                break
            rospy.loginfo("SIFT running")
            with self.mtx:
                gray = self.curr_img
                H0 = self.H
            cnt = cv2.perspectiveTransform(self.template_corners, H0)
            bbox = cv2.boundingRect(cnt)
            bbox = adjust_bbox(bbox)
            x, y, w, h = bbox
            crop = gray[int(y):int(y + h), int(x):int(x + w)]
            # cv2.imshow("crop", crop)
            kp, des = self.f2d_extractor.detectAndCompute(crop, mask=None)
            if kp is None or len(kp) < min_inliers:
                continue
            kp = self.kp2array(kp, x_offset=x, y_offset=y)
            pts0, pts = self.match(kp0, kp, des0, des)
            if pts is None or len(pts) < min_inliers:
                continue
            H0_f, pts0, pts = self.reject_with_H(pts0, pts)
            # print(len(pts))
            if pts is None or len(pts) < min_inliers:
                continue
            with self.mtx:
                H1 = self.H
                H = np.matmul(H1, np.matmul(np.linalg.inv(H0), H0_f))
                self.H = H
                self.curr_co_pts = cv2.perspectiveTransform(self.curr_pts, np.linalg.inv(H))
            continue
        return
    def match(self, kp0, kp, des0, des):
        matches = self.fd_matcher.knnMatch(des0, des, k=2)
        good = []
        for m,n in matches:
            if m.distance < 0.5*n.distance:
                good.append(m)
        pts0 = np.reshape(np.float32([kp0[m.queryIdx] for m in good]), (-1, 1, 2))
        pts = np.reshape(np.float32([kp[m.trainIdx] for m in good]), (-1, 1, 2))
        return pts0, pts
    def reject_with_H(self, pts0, pts):
        H, status = cv2.findHomography(pts0, pts, method=cv2.RANSAC)
        pts0 = np.reshape(pts0[status==1], (-1, 1, 2))
        pts = np.reshape(pts[status==1], (-1, 1, 2))
        H, _ = cv2.findHomography(pts0, pts, method=0)
        return H, pts0, pts
    def kp2array(self, kp, x_offset=0, y_offset=0):
        pts = np.reshape(np.float32([[p.pt[0] + x_offset, p.pt[1] + y_offset] for p in kp]), (-1, 1, 2))
        return pts

def main():
    cap = cv2.VideoCapture('testset/test.mp4')
    if not cap.isOpened():
        print("Video cannot be opened!")
        return
    detector = LogoDetector()
    while True:
        ret, src = cap.read()
        if not ret:
            print("End of the video!")
            break
        errno, corners = detector.search(src)
        if errno == 0:
            src = cv2.polylines(src, [np.reshape(np.int32(corners), (4, 2))], 1, (0, 0, 255), 2)
            src = draw_pts(src, detector.curr_pts)
        else:
            print(errno)
        cv2.imshow("src", src)
        key = cv2.waitKey(0)
        if key == ord('q'):
            detector.reset()
            print("Manual shut down!")
            break

if __name__ == "__main__":
    rospy.init_node("detector")
    main()