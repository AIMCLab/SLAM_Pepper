import os
import argparse
import cv2

import utils

parser = argparse.ArgumentParser()
parser.add_argument('image_dir', metavar='IMAGE_DIR',
                    help='the directory of images')
parser.add_argument('--fps', '-f', type=int, default=20,
                    help='the video playback speed')
args = parser.parse_args()


if __name__ == '__main__':
    img_list = utils.get_image_list(args.image_dir)
    for (i, fname) in enumerate(img_list):
        print('Frame [%d/%d] % s ...' % (i + 1, len(img_list), fname))
        img = cv2.imread(os.path.join(args.image_dir, fname))

        cv2.imshow('image', img)
        cv2.waitKey(int(1000 / args.fps))

    cv2.waitKey(0)
