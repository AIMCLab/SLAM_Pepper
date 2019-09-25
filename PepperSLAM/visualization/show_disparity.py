import cv2
import argparse
import numpy as np

import utils

parser = argparse.ArgumentParser()
parser.add_argument('disparity', metavar="DISPARITY",
                    help='the disparity data')
parser.add_argument('--width', '-W', type=int, help='the image width')
parser.add_argument('--height', '-H', type=int, help='the image height')
parser.add_argument('--max-disp', '-m', type=int,
                    help='the maximum disparity, default is the maximum value of input disparity')
parser.add_argument('--output', '-o', default='output',
                    help='the output name')
parser.add_argument('--rgb', '-c', action='store_true',
                    help='colorize disparity map')
args = parser.parse_args()


if __name__ == '__main__':
    disp = utils.load_disparity_binary(args.disparity).astype(np.float32)
    if args.width:
        disp = disp.reshape(-1, args.width)
    elif args.height:
        disp = disp.reshape(args.height, -1)
    else:
        print('Must give one of the width or height!')
        exit(0)

    if args.max_disp:
        max_disp = args.max_disp
    else:
        max_disp = disp.max()
    print('Max disparity: %d / %d' % (disp.max(), max_disp))

    output = utils.disparity_vis(disp, max_disp, rgb=args.rgb)
    cv2.imwrite(args.output + '.png', output)
