import cv2
import argparse
import numpy as np

import utils
from utils.pointcloud import PointCloud

parser = argparse.ArgumentParser()
parser.add_argument('--calib', '-c', default='camera_calib.yml',
                    help='the camera calibration results')
parser.add_argument('--rgb', help='the RGB image')
parser.add_argument('--depth', help='the depth data (*.png, *.data)')
parser.add_argument('--disp', help='the disparity data (*.png, *.data)')
parser.add_argument('--output', '-o', default='pointcloud.ply',
                    help='the output point cloud file')
args = parser.parse_args()


if __name__ == '__main__':
    rgb = cv2.imread(args.rgb)
    fs = cv2.FileStorage(args.calib, cv2.FileStorage_READ)
    calib_height = fs.getNode('image_size').mat()[1][0]
    height = rgb.shape[0]
    roi = (0, 0, rgb.shape[1], rgb.shape[0])

    if args.depth:
        if args.depth.endswith('.png'):
            depth = cv2.imread(args.depth, cv2.IMREAD_GRAYSCALE)
        elif args.depth.endswith('.data'):
            depth = utils.load_disparity_binary(
                args.depth).reshape(height, -1).astype(np.float32)
        else:
            print("Unknown depth data format")
            exit(0)

        scale = height / calib_height
        camera_matrix = fs.getNode('K').mat()
        camera_matrix = utils.scale_camera_matrix(camera_matrix, scale)

        pc = PointCloud.from_depth(
            depth, rgb, 40, camera_matrix, clip=(0, 255), roi=roi, scale=1.0)

    elif args.disp:
        if args.disp.endswith('.png'):
            disp = cv2.imread(args.disp, cv2.IMREAD_GRAYSCALE)
        elif args.disp.endswith('.data'):
            disp = utils.load_disparity_binary(
                args.disp).reshape(height, -1).astype(np.float32)
        else:
            print("Unknown disparity data format")
            exit(0)

        scale = height / calib_height
        Q = fs.getNode('Q').mat()
        Q = utils.scale_Q(Q, scale)

        pc = PointCloud.from_disparity(
            disp, rgb, Q, clip=(10, 255), roi=roi, scale=1.0)

    if pc:
        pc.save(args.output)
    else:
        print('No depth or disparity image!')
