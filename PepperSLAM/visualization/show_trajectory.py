import argparse
import numpy as np
from vispy import scene
from vispy.color import Color
from vispy.scene import visuals

from utils import trajectory

parser = argparse.ArgumentParser()
parser.add_argument('groundtruth_file', metavar='GROUNDTRUTH_FILE',
                    help='the groundtruth trajectory file (format: "timestamp tx ty tz qx qy qz qw")')
parser.add_argument('estimated_file', metavar='ESTIMATED_FILE', nargs='?',
                    help='the estimated trajectory file (format: "timestamp tx ty tz qx qy qz qw")')
parser.add_argument('--strip', type=int, default=1,
                    help='the groundtruth trajectory strip')
parser.add_argument('--offset', type=float, default=0.0,
                    help='time offset added to the timestamps of the second file (default: 0.0)')
parser.add_argument('--scale', '-s', type=float, default=1.0,
                    help='scaling factor for the second trajectory (default: 1.0)')
parser.add_argument('--max_difference', type=float, default=0.02,
                    help='maximally allowed time difference for matching entries (default: 0.02)')
args = parser.parse_args()


canvas = scene.SceneCanvas(keys='interactive', show=True)
view = canvas.central_widget.add_view()
view.camera = scene.ArcballCamera(fov=60)
visuals.XYZAxis(parent=view.scene)


def add_trajectory(points, color='white', strip=1, show_lines=True, show_arrows=True):
    arrows = np.hstack(
        (points[:-1:strip], points[1::strip])) if show_arrows else None
    visuals.Arrow(points if show_lines else None, arrows=arrows,
                  arrow_size=5, arrow_color=color, parent=view.scene)


if __name__ == '__main__':
    first_list = trajectory.read_file_list(args.groundtruth_file)
    if args.estimated_file is None:
        points_g = np.array(
            [[float(value) for value in first_list[t][0:3]] for t in sorted(first_list.keys())])
        add_trajectory(points_g, 'red', args.strip)
    else:
        second_list = trajectory.read_file_list(args.estimated_file)
        matches = trajectory.associate(
            first_list, second_list, args.offset, args.max_difference)

        first_points_matched = np.array(
            [[float(value) for value in first_list[a][0:3]] for a, b in matches])
        second_points_matched = np.array(
            [[float(value)*float(args.scale) for value in second_list[b][0:3]] for a, b in matches])

        rot, trans, trans_error = trajectory.align_trajectories(
            first_points_matched, second_points_matched)
        print('ATE RMSE: %f' %
              np.sqrt(np.dot(trans_error, trans_error) / len(trans_error)))

        points_g = np.array(
            [[float(value) for value in first_list[t][0:3]] for t in sorted(first_list.keys())])
        points_e = np.array(
            [[float(value)*float(args.scale) for value in second_list[t][0:3]] for t in sorted(second_list.keys())])
        points_e = (rot.dot(points_e.T) + trans).T

        add_trajectory(points_g, 'red', args.strip)
        add_trajectory(points_e, 'blue', 1)

    canvas.app.run()
