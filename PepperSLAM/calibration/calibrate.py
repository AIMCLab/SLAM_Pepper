import os
import sys
import cv2
import math
import argparse
import numpy as np
from utils import get_image_list, find_chess_board_corners, undistort_image

PATTERN_SIZE = (9, 6)

parser = argparse.ArgumentParser()
parser.add_argument('base_dir', metavar='BASE_DIR',
                    help='the base directory of images')
parser.add_argument('-d', '--debug', action='store_true',
                    help='generate debug images into DEBUG_DIR')
parser.add_argument('--input_dir', default='left',
                    help='the directory used to store input images')
parser.add_argument('--debug_dir', default='debug',
                    help='the directory used to generate debug images')
parser.add_argument('--square_size', default=1.0,
                    help='the chess board square size')
parser.add_argument('--output', '-o', default='camera_calib.yml',
                    help='the calibration results')
args = parser.parse_args()


def compute_reprojection_errors(obj_points, img_points, rvecs, tvecs, camera_matrix, dist_coefs):
    errs = []
    tot_err = 0
    tot_points = 0
    for i in range(len(obj_points)):
        points, _ = cv2.projectPoints(obj_points[i], rvecs[i],
                                      tvecs[i], camera_matrix, dist_coefs)
        points = points.reshape(-1, 2)

        n = len(obj_points[i])
        err = cv2.norm(img_points[i], points, cv2.NORM_L2)

        errs.append(math.sqrt(err * err / n))
        tot_err += err * err
        tot_points += n

    return math.sqrt(tot_err / tot_points), errs


def calibrate(obj_points, img_points, image_size):
    flags = 0
    flags |= cv2.CALIB_FIX_K4
    flags |= cv2.CALIB_FIX_K5
    rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, image_size, None, None, flags=flags)

    return rms, camera_matrix, dist_coefs, rvecs, tvecs


if __name__ == '__main__':
    basedir = args.base_dir
    input_dir = os.path.join(basedir, args.input_dir)
    debug_dir = os.path.join(basedir, args.debug_dir)

    if args.debug and not os.path.exists(debug_dir):
        os.mkdir(debug_dir)

    pattern_points = np.zeros((np.prod(PATTERN_SIZE), 3), np.float32)
    pattern_points[:, :2] = np.indices(PATTERN_SIZE).T.reshape(-1, 2)
    pattern_points *= args.square_size

    obj_points, img_points = [], []

    image_list = get_image_list(input_dir)
    selected_images = []
    pre_img = None
    for i, fname in enumerate(image_list):
        print('[%d/%d] Processing % s ...' % (i + 1, len(image_list), fname))
        img = cv2.imread(os.path.join(input_dir, fname))

        if np.array_equal(img, pre_img):
            continue
        pre_img = img.copy()

        corners = find_chess_board_corners(img, PATTERN_SIZE)
        image_size = img.shape[:2][::-1]

        if corners is not None:
            obj_points.append(pattern_points)
            img_points.append(corners)
            selected_images.append(fname)

            if args.debug:
                cv2.drawChessboardCorners(img, PATTERN_SIZE, corners, True)
                cv2.imwrite(os.path.join(debug_dir, 'corners_' + fname), img)

    print('Calibrating %d images ...' % len(obj_points))

    rms, camera_matrix, dist_coefs, rvecs, tvecs = calibrate(
        obj_points, img_points, image_size)
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coefs, image_size, 1, image_size)

    print(rms)
    print(camera_matrix)
    print(dist_coefs)
    print(newcameramtx)
    print(roi)

    fs = cv2.FileStorage(args.output, cv2.FileStorage_WRITE)
    fs.write('image_size', image_size)
    fs.write('K', camera_matrix)
    fs.write('D', dist_coefs)
    fs.release()

    if args.debug:
        tot_err, errs = compute_reprojection_errors(
            obj_points, img_points, rvecs, tvecs, camera_matrix, dist_coefs)
        print('Avg Reprojection Error:', tot_err)
        print('Per View Reprojection Errors:')
        for i, err in enumerate(errs):
            print(i, selected_images[i], err)

    if args.debug:
        for i, fname in enumerate(image_list):
            print('[%d/%d] Undistorting % s ...' %
                  (i + 1, len(image_list),  fname))
            img = cv2.imread(os.path.join(input_dir, fname))
            dst = undistort_image(img, camera_matrix,
                                  dist_coefs, None, newcameramtx)

            x, y, w, h = roi
            cv2.rectangle(dst, (x, y), (x + w - 1, y + h - 1), (0, 0, 255), 1)

            cv2.imwrite(os.path.join(debug_dir, 'undistort_' + fname), dst)
            cv2.imshow('rectified', dst)
            cv2.waitKey(20)

    cv2.waitKey(0)
