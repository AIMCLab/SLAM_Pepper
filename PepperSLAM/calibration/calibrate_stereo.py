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
parser.add_argument('--left_dir', default='left',
                    help='the directory used to store stereo left images')
parser.add_argument('--right_dir', default='right',
                    help='the directory used to store stereo right images')
parser.add_argument('--debug_dir', default='debug',
                    help='the directory used to generate debug images')
parser.add_argument('--square_size', type=float, default=3.75,
                    help='the chess board square size')
parser.add_argument('--output', '-o', help='the calibration results')
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

    return rms, camera_matrix, dist_coefs


def calibrate_stereo(obj_points, img_points_l, img_points_r, image_size):
    rms_l, camera_matrix_l, dist_coefs_l = calibrate(
        obj_points, img_points_l, image_size)
    rms_r, camera_matrix_r, dist_coefs_r = calibrate(
        obj_points, img_points_r, image_size)

    flags = 0
    flags |= cv2.CALIB_FIX_K4
    flags |= cv2.CALIB_FIX_K5
    flags |= cv2.CALIB_USE_INTRINSIC_GUESS

    term = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
    rms, camera_matrix_l, dist_coefs_l, camera_matrix_r, dist_coefs_r, R, T, E, F = cv2.stereoCalibrate(
        obj_points, img_points_l, img_points_r,
        camera_matrix_l, dist_coefs_l, camera_matrix_r, dist_coefs_r, image_size,
        criteria=term, flags=flags)

    return rms, camera_matrix_l, dist_coefs_l, camera_matrix_r, dist_coefs_r, R, T


if __name__ == '__main__':
    basedir = args.base_dir
    left_dir = os.path.join(basedir, args.left_dir)
    right_dir = os.path.join(basedir, args.right_dir)
    debug_dir = os.path.join(basedir, args.debug_dir)

    if args.debug and not os.path.exists(debug_dir):
        os.mkdir(debug_dir)

    pattern_points = np.zeros((np.prod(PATTERN_SIZE), 3), np.float32)
    pattern_points[:, :2] = np.indices(PATTERN_SIZE).T.reshape(-1, 2)
    pattern_points *= args.square_size

    obj_points = []
    img_points_l = []
    img_points_r = []

    image_list = get_image_list(left_dir)
    pre_img_l, pre_img_r = None, None
    for i, fname in enumerate(image_list):
        print('[%d/%d] Processing % s ...' % (i + 1, len(image_list), fname))
        img_l = cv2.imread(os.path.join(left_dir, fname))
        img_r = cv2.imread(os.path.join(right_dir, fname))

        if np.array_equal(img_l, pre_img_l) and np.array_equal(img_r, pre_img_r):
            continue
        pre_img_l, pre_img_r = img_l.copy(), img_r.copy()

        corners_l = find_chess_board_corners(img_l, PATTERN_SIZE)
        corners_r = find_chess_board_corners(img_r, PATTERN_SIZE)
        image_size = img_l.shape[:2][::-1]

        if corners_l is not None and corners_r is not None:
            obj_points.append(pattern_points)
            img_points_l.append(corners_l)
            img_points_r.append(corners_r)
            if args.debug:
                cv2.drawChessboardCorners(img_l, PATTERN_SIZE, corners_l, True)
                cv2.drawChessboardCorners(img_r, PATTERN_SIZE, corners_r, True)
                output = np.concatenate((img_l, img_r), axis=1)
                cv2.imwrite(os.path.join(
                    debug_dir, 'corners_' + fname), output)

    print('Calibrating %d images ...' % len(obj_points))

    rms, camera_matrix_l, dist_coefs_l, camera_matrix_r, dist_coefs_r, R, T = calibrate_stereo(
        obj_points, img_points_l, img_points_r, image_size)

    R1, R2, P1, P2, Q, roi_l, roi_r = cv2.stereoRectify(camera_matrix_l, dist_coefs_l,
                                                        camera_matrix_r, dist_coefs_r,
                                                        image_size, R, T, alpha=0)

    print('Avg Reprojection Error:', rms)
    print('R, T:')
    print(R)
    print(T)
    print('Left:')
    print(camera_matrix_l)
    print(dist_coefs_l)
    print(R1)
    print(P1)
    print(roi_l)
    print('Right:')
    print(camera_matrix_r)
    print(dist_coefs_r)
    print(R2)
    print(P2)
    print(roi_r)

    if args.output:
        fs = cv2.FileStorage(args.output, cv2.FileStorage_WRITE)
        fs.write('image_size', image_size)
        fs.write('K1', camera_matrix_l)
        fs.write('D1', dist_coefs_l)
        fs.write('K2', camera_matrix_r)
        fs.write('D2', dist_coefs_r)
        fs.write('R', R)
        fs.write('T', T)
        fs.write('R1', R1)
        fs.write('P1', P1)
        fs.write('R2', R2)
        fs.write('P2', P2)
        fs.write('Q', Q)
        fs.release()

    if args.debug:
        for i, fname in enumerate(image_list):
            print('[%d/%d] Undistorting % s ...' %
                  (i + 1, len(image_list),  fname))
            img_l = cv2.imread(os.path.join(left_dir, fname))
            img_r = cv2.imread(os.path.join(right_dir, fname))

            dst_l = cv2.undistort(img_l, camera_matrix_l, dist_coefs_l)
            dst_r = cv2.undistort(img_r, camera_matrix_r, dist_coefs_r)
            output = np.concatenate((dst_l, dst_r), axis=1)
            h, w = output.shape[:2]
            for i in range(h // 10, h, h // 10):
                cv2.line(output, (0, i), (w, i), (0, 0, 255))
            cv2.imwrite(os.path.join(
                debug_dir, 'undistorted_' + fname), output)
            cv2.imshow('undistorted', output)

            dst_l = undistort_image(
                img_l, camera_matrix_l, dist_coefs_l, R1, P1)
            dst_r = undistort_image(
                img_r, camera_matrix_r, dist_coefs_r, R2, P2)
            output = np.concatenate((dst_l, dst_r), axis=1)
            h, w = output.shape[:2]
            for i in range(h // 10, h, h // 10):
                cv2.line(output, (0, i), (w, i), (0, 255, 0))

            cv2.imwrite(os.path.join(
                debug_dir, 'rectified_' + fname), output)
            cv2.imshow('rectified', output)

            cv2.waitKey(20)

        cv2.waitKey(0)
