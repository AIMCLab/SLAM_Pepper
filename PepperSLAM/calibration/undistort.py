import os
import sys
import cv2
import math
import argparse
import numpy as np

import utils

PATTERN_SIZE = (9, 6)

parser = argparse.ArgumentParser()
parser.add_argument('input', metavar='INPUT',
                    help='the input image')
parser.add_argument('--calib', '-c', default="camera_calib.yml",
                    help='the camera calibration results')
parser.add_argument('--rectify', '-r', action="store_true",
                    help='rectify undistorted image')
parser.add_argument('--output', '-o', default='output.png',
                    help='the undistorted image')
args = parser.parse_args()


if __name__ == '__main__':
    img = cv2.imread(args.input)
    height, width = img.shape[0], img.shape[1]

    fs = cv2.FileStorage(args.calib, cv2.FileStorage_READ)
    calib_height = fs.getNode('image_size').mat()[1][0]

    camera_matrix = fs.getNode('K').mat()
    if camera_matrix is None:
        camera_matrix = fs.getNode('K1').mat()
    if camera_matrix is None:
        camera_matrix = np.array([
            [1, 0, img.shape[1]],
            [0, 1, img.shape[0]]
            [0, 0, 1]
        ])

    dist_coefs = fs.getNode('D').mat()
    if dist_coefs is None:
        dist_coefs = fs.getNode('D1').mat()
    if dist_coefs is None:
        dist_coefs = np.zeros(5)

    scale = height / calib_height
    camera_matrix = utils.scale_camera_matrix(camera_matrix, scale)

    if args.rectify:
        R = fs.getNode('R1').mat()
        P = fs.getNode('P1').mat()
    else:
        P = None
        R = None
    if P is None:
        P = camera_matrix
    else:
        P = utils.scale_camera_matrix(P, scale)

    dst = utils.undistort_image(
        img, camera_matrix, dist_coefs, R, P, interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(args.output, dst)
    # cv2.imshow('undistorted', dst)
    cv2.waitKey(0)
