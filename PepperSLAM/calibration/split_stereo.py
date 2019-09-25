import os
import sys
import cv2
from utils import get_image_list, split_stereo_image

INPUT_DIR_NAME = 'rgb'
LEFT_DIR_NAME = 'left'
RIGHT_DIR_NAME = 'right'

if __name__ == '__main__':
    basedir = sys.argv[1]
    input_dir = os.path.join(basedir, INPUT_DIR_NAME)
    left_dir = os.path.join(basedir, LEFT_DIR_NAME)
    right_dir = os.path.join(basedir, RIGHT_DIR_NAME)

    if not os.path.exists(left_dir):
        os.mkdir(left_dir)
    if not os.path.exists(right_dir):
        os.mkdir(right_dir)

    image_list = get_image_list(input_dir)
    for i, fname in enumerate(image_list):
        print("[%d/%d] Spliting % s ..." % (i + 1, len(image_list), fname))
        im = cv2.imread(os.path.join(input_dir, fname))
        im_l, im_r = split_stereo_image(im)
        cv2.imwrite(os.path.join(left_dir, fname), im_l)
        cv2.imwrite(os.path.join(right_dir, fname), im_r)
