import cv2
import argparse
import numpy as np
from vispy import scene
from vispy.color import Color
from vispy.scene import visuals

import utils
from utils.pointcloud import PointCloud

parser = argparse.ArgumentParser()
parser.add_argument('--calib', '-c', default='camera_calib.yml',
                    help='the camera calibration results')
parser.add_argument('--rgb', required=True, help='the RGB image')
parser.add_argument('--disp', required=True,
                    help='the disparity data (*.png, *.data)')
parser.add_argument('--output', '-o', default='pointcloud.ply',
                    help='the output point cloud file')
args = parser.parse_args()


canvas = scene.SceneCanvas(keys='interactive', show=True)
view = canvas.central_widget.add_view()
view.camera = scene.ArcballCamera(fov=60)
visuals.XYZAxis(parent=view.scene)

if __name__ == '__main__':
    rgb = cv2.imread(args.rgb)
    fs = cv2.FileStorage(args.calib, cv2.FileStorage_READ)
    calib_height = fs.getNode('image_size').mat()[1][0]
    height = rgb.shape[0]
    roi = (0, 0, rgb.shape[1], rgb.shape[0])

    if args.disp:
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
            disp, rgb, Q, clip=(3000, 1e9), roi=roi, scale=1.0)

        points = pc.points
        colors = pc.colors / 255.0

        scatter = visuals.Markers()
        scatter.set_data(points,
                         edge_width=None,
                         edge_width_rel=0,
                         face_color=colors, size=2)
        view.add(scatter)

    canvas.app.run()
