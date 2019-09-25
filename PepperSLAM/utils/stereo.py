import cv2
import numpy as np

import utils
from utils.pointcloud import PointCloud


class StereoMatcher(object):
    def __init__(self, camera_params_file, img_size):
        fs = cv2.FileStorage(camera_params_file, cv2.FileStorage_READ)
        calib_img_size = fs.getNode('image_size').mat()
        self.width, self.height = img_size
        self.K1 = fs.getNode('K1').mat()
        self.D1 = fs.getNode('D1').mat()
        self.K2 = fs.getNode('K2').mat()
        self.D2 = fs.getNode('D2').mat()
        self.R = fs.getNode('R').mat()
        self.T = fs.getNode('T').mat()
        fs.release()

        scale = img_size[0] / calib_img_size[0][0]
        self.K1 = utils.scale_camera_matrix(self.K1, scale)
        self.K2 = utils.scale_camera_matrix(self.K2, scale)
        self.R1, self.R2, self.P1, self.P2, self.Q, roi_l, roi_r = cv2.stereoRectify(
            self.K1, self.D1, self.K2, self.D2, img_size, self.R, self.T)
        self.Q_saved = self.Q

    def create_matcher(self, min_disp, num_disp, block_size, mode='sgbm', use_disp_filter=False):
        self.min_disp = min_disp
        self.num_disp = num_disp
        self.max_disp = min_disp + num_disp
        mode = {
            'sgbm': cv2.StereoSGBM_MODE_SGBM,
            'sgbm3way': cv2.StereoSGBM_MODE_SGBM_3WAY,
            'hh': cv2.StereoSGBM_MODE_HH,
            'hh4': cv2.StereoSGBM_MODE_HH4,
        }[mode]
        self.left_matcher = cv2.StereoSGBM_create(minDisparity=min_disp,
                                                  numDisparities=num_disp,
                                                  blockSize=block_size,
                                                  P1=8*3*block_size**2,
                                                  P2=64*3*block_size**2,
                                                  preFilterCap=63,
                                                  uniquenessRatio=15,
                                                  disp12MaxDiff=1,
                                                  speckleWindowSize=100,
                                                  speckleRange=1,
                                                  mode=mode,
                                                  )

        if use_disp_filter:
            self.right_matcher = cv2.ximgproc.createRightMatcher(
                self.left_matcher)
            self.disp_filter = cv2.ximgproc.createDisparityWLSFilter(
                matcher_left=self.left_matcher)
            self.disp_filter.setLambda(1000)
            self.disp_filter.setSigmaColor(0.5)

            # self.left_matcher.setDisp12MaxDiff(1)
            self.left_matcher.setSpeckleWindowSize(100)
            self.left_matcher.setSpeckleRange(1)
            # self.right_matcher.setDisp12MaxDiff(1)
            self.right_matcher.setSpeckleWindowSize(100)
            self.right_matcher.setSpeckleRange(1)
        else:
            self.right_matcher = None
            self.disp_filter = None

    def undistort_stereo_images(self, img_l, img_r):
        img_l = utils.undistort_image(
            img_l, self.K1, self.D1, self.R1, self.P1)
        img_r = utils.undistort_image(
            img_r, self.K2, self.D2, self.R2, self.P2)
        return img_l, img_r

    def compute_disparity(self, img_l, img_r, undistort=False, roi=None):
        assert self.left_matcher is not None
        if undistort:
            img_l, img_r = self.undistort_stereo_images(img_l, img_r)
        if roi is not None:
            x, y, w, h = roi
            img_l = img_l[y:y+h, x:x+w]
            img_r = img_r[y:y+h, x:x+w]
            self.Q[0, 3] = x * self.Q_saved[0, 0]
            self.Q[1, 3] = y * self.Q_saved[1, 1]
        self.img_l, self.img_r = img_l, img_r

        self.disp_l = self.left_matcher.compute(img_l, img_r)
        if self.right_matcher is not None:
            self.disp_r = self.right_matcher.compute(img_r, img_l)
        else:
            self.disp_r = None

        if self.disp_filter is not None:
            self.disp_f = self.disp_filter.filter(
                self.disp_l, img_l, None, self.disp_r)
            self.disp = self.disp_f
        else:
            self.disp = self.disp_l
        self.disp = self.disp.clip(self.min_disp*16, self.max_disp*16)

    def get_roi(self):
        if self.disp_filter is not None:
            return self.disp_filter.getROI()
        else:
            return (self.max_disp, 0, self.width-self.max_disp, self.height)

    def get_pointcloud(self, clip=None, roi=None, scale=1.0):
        assert self.disp is not None and self.img_l is not None
        roi = self.get_roi() if roi is None else roi
        clip = (self.min_disp*16, self.max_disp*16) if clip is None else clip
        return PointCloud.from_disparity(self.disp, self.img_l, self.Q,
                                         clip=clip, roi=roi, scale=scale)

    def get_disparity_vis(self, max_disp=None, rgb=False):
        assert self.disp is not None
        if max_disp is None:
            max_disp = self.max_disp*16
        return utils.disparity_vis(self.disp, max_disp, rgb=rgb)

    def get_depth(self, max_depth):
        assert self.disp is not None
        depth = np.where(self.disp > max(0, self.disp.min()),
                         max_depth / self.disp, 0.0)
        return depth * (max_depth / depth.max())

    def get_depth_vis(self, max_depth=None, rgb=False):
        assert self.disp is not None
        if max_depth is None:
            max_depth = self.Q[2, 3] / self.Q[3, 2]
        self.depth = self.get_depth(max_depth)
        return utils.disparity_vis(self.depth, max_depth, rgb=rgb)
