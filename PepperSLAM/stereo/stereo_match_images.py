import os
import argparse
import cv2
import numpy as np

import utils
from utils.stereo import StereoMatcher

parser = argparse.ArgumentParser()
parser.add_argument('base_dir', metavar='BASE_DIR',
                    help='the base directory of images')
parser.add_argument('--input_dir', default='rgb',
                    help='the directory used to store stereo images')
parser.add_argument('--disp_dir', default='disparity',
                    help='the directory used to store disparity images')
parser.add_argument('--calib', '-c', default='camera_calib.yml',
                    help='the camera calibration results')
parser.add_argument('--save-undistort', '-su', action='store_true',
                    help='save undistorted images to the left/right directory')

parser.add_argument('--filter', '-f', action='store_true',
                    help='use the disparity map post-filtering')
parser.add_argument('--min-disp', type=int, default=0,
                    help='minimum possible disparity value used in SGBM')
parser.add_argument('--num-disp', type=int,
                    help='maximum disparity minus minimum disparity used in SGBM')
parser.add_argument('--blocksize', '-bs', type=int, default=3,
                    help='matched block size used in SGBM')
parser.add_argument('--mode', '-m', choices=['sgbm', 'sgbm3way', 'hh', 'hh4'],
                    default='sgbm', help='the mode used in SGBM')
args = parser.parse_args()


if __name__ == '__main__':
    basedir = args.base_dir
    input_dir = os.path.join(basedir, args.input_dir)
    disp_dir = os.path.join(basedir, args.disp_dir)

    if not os.path.exists(disp_dir):
        os.mkdir(disp_dir)

    if args.save_undistort:
        left_dir = os.path.join(basedir, 'left')
        right_dir = os.path.join(basedir, 'right')
        if not os.path.exists(left_dir):
            os.mkdir(left_dir)
        if not os.path.exists(right_dir):
            os.mkdir(right_dir)

    img_list = utils.get_image_list(input_dir)
    img_size = cv2.imread(os.path.join(input_dir, img_list[0])).shape[:2][::-1]
    img_size = (img_size[0] // 2, img_size[1])
    roi = (0, 0, img_size[0], img_size[1])

    min_disp = args.min_disp
    if args.num_disp is None:
        num_disp = (img_size[0] // 8 + 15) & -16
    else:
        num_disp = args.num_disp

    matcher = StereoMatcher(args.calib, img_size)
    matcher.create_matcher(
        min_disp, num_disp, args.blocksize, args.mode, use_disp_filter=args.filter)

    for (i, fname) in enumerate(img_list):
        print('[%d/%d] Processing % s ...' % (i + 1, len(img_list), fname))
        img = cv2.imread(os.path.join(input_dir, fname))
        img_l, img_r = utils.split_stereo_image(img)
        roi = (10, 0, 280, 180)

        matcher.compute_disparity(img_l, img_r, undistort=True, roi=roi)
        utils.save_disparity_binary(
            os.path.join(disp_dir, fname[:-4] + '.data'), matcher.disp)
        disp_vis = matcher.get_disparity_vis(matcher.max_disp*16, rgb=False)
        cv2.imwrite(os.path.join(disp_dir, fname[:-4] + '.png'), disp_vis)

        if args.save_undistort:
            cv2.imwrite(os.path.join(left_dir, fname), matcher.img_l)
            cv2.imwrite(os.path.join(right_dir, fname), matcher.img_r)

        cv2.imshow('image', np.hstack((matcher.img_l, matcher.img_r)))
        cv2.imshow('disparity', disp_vis)
        cv2.waitKey(20)

    cv2.waitKey(0)
