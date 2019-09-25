import argparse
import cv2
import numpy as np

from utils.stereo import StereoMatcher

parser = argparse.ArgumentParser()
parser.add_argument('--calib', '-c', default='camera_calib.yml',
                    help='the camera calibration results')
parser.add_argument('--left', required=True, help='the left image')
parser.add_argument('--right', required=True, help='the right image')
parser.add_argument('--no-undistort', '-nu', action='store_true',
                    help='do not need to undistort images')
parser.add_argument('--disp', default='disp.png',
                    help='the output disparity image')
parser.add_argument('--depth', help='the output depth image')
parser.add_argument('--pointcloud', '-p', help='the output point cloud file')

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
    img_l = cv2.imread(args.left)
    img_r = cv2.imread(args.right)
    img_size = img_l.shape[:2][::-1]

    matcher = StereoMatcher(args.calib, img_size)

    min_disp = args.min_disp
    if args.num_disp is None:
        num_disp = (img_size[0] // 8 + 15) & -16
    else:
        num_disp = args.num_disp

    matcher.create_matcher(
        min_disp, num_disp, args.blocksize, args.mode, use_disp_filter=args.filter)
    matcher.compute_disparity(img_l, img_r, undistort=not args.no_undistort)

    if args.pointcloud:
        matcher.get_pointcloud(
            clip=(min_disp*16, matcher.max_disp*16), scale=1.0).save(args.pointcloud)
    if args.disp:
        disp_vis = matcher.get_disparity_vis(matcher.max_disp*16, rgb=False)
        cv2.imwrite(args.disp, disp_vis)
    if args.depth:
        depth_vis = matcher.get_depth_vis(rgb=False)
        cv2.imwrite(args.depth, depth_vis)
