import os
import cv2
import struct
import numpy as np


def get_image_list(path):
    images = filter(lambda fname: fname.endswith('.png'), os.listdir(path))
    return sorted(images, key=lambda fname: int(fname[:-4]))


def split_stereo_image(im):
    width = im.shape[1]
    im_l = im[:, :width // 2]
    im_r = im[:, width // 2:]
    return im_l, im_r


def find_chess_board_corners(img, pattern_size):
    found, corners = cv2.findChessboardCorners(img, pattern_size)
    if found:
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        cv2.cornerSubPix(img_gray, corners, (11, 11), (-1, -1), term)
        return corners.reshape(-1, 2)
    else:
        print('chessboard not found!')
        return None


def undistort_image(img, camera_matrix, dist_coefs, R, P, interpolation=cv2.INTER_LINEAR):
    mapx, mapy = cv2.initUndistortRectifyMap(
        camera_matrix, dist_coefs, R, P, img.shape[:2][::-1], cv2.CV_32FC1)
    dst = cv2.remap(img, mapx, mapy, interpolation)
    # dst = cv2.undistort(img, camera_matrix, dist_coefs, None, P)
    return dst


def disparity_vis(disp, max_disp, rgb=True):
    mi = 0
    img = ((disp - mi) * 256.0 / max_disp).clip(0, 255).astype(np.uint8)
    if rgb:
        img = cv2.applyColorMap(img, cv2.COLORMAP_JET)
    return img


def save_disparity_binary(filename, disp):
    with open(filename, 'wb') as f:
        for data in disp.flatten():
            f.write(struct.pack('H', data))


def load_disparity_binary(filename):
    with open(filename, 'rb') as f:
        data = f.read()
    return np.array(struct.unpack('H' * (len(data) // 2), data), np.uint16)


def scale_camera_matrix(camera_matrix, scale=1.0):
    factor = np.array([scale, scale, 1]).reshape(-1, 1)
    return camera_matrix * factor


def scale_Q(Q, scale=1.0):
    Q[0][3] *= scale
    Q[1][3] *= scale
    Q[2][3] *= scale
    return Q
