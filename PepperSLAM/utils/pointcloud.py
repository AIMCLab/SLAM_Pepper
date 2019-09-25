import cv2
import numpy as np

np.seterr(divide='ignore')


class PointCloud(object):
    def __init__(self, points, colors=None):
        if colors is not None:
            assert points.shape == colors.shape
        self.points = points
        self.colors = colors

    @staticmethod
    def from_depth(depth, rgb=None, fb=100.0, camera_matrix=None, roi=None, clip=None, scale=1.0):
        # depth = fb / disp
        if rgb is not None:
            assert depth.shape[:2] == rgb.shape[:2]
        if camera_matrix is None:
            h, w = depth.shape[:2]
            ifx, ify = 1 / (w * 0.8), 1 / (h * 0.8)
            cx, cy = w * 0.5, h * 0.5
        else:
            ifx, ify = 1 / camera_matrix[0, 0], 1 / camera_matrix[1, 1]
            cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
        disp = np.where(depth > 0, float(fb) / depth, 0.0).astype(np.float32)
        Q = np.float32([[ifx, 0, 0, -cx * ifx],
                        [0, ify, 0, -cy * ify],
                        [0, 0, 0, 1],
                        [0, 0, 1, 0]])
        if clip is not None:
            l = 1e9 if clip[1] == 0 else float(fb) / clip[1]
            r = 1e9 if clip[0] == 0 else float(fb) / clip[0]
            clip = (l, r)
        return PointCloud.from_disparity(disp, rgb, Q, roi=roi, clip=clip, scale=scale)

    @staticmethod
    def from_disparity(disp, rgb=None, Q=None, fb=None, roi=None, clip=None, scale=1.0):
        if rgb is not None:
            assert disp.shape[:2] == rgb.shape[:2]
        if Q is None:
            h, w = disp.shape[:2]
            if fb is None:
                fb = w * 0.8
            Q = np.float32([[1, 0, 0, -0.5*w],
                            [0, 1, 0, -0.5*h],
                            [0, 0, 0, fb],
                            [0, 0, 1, 0]])
        if roi is not None:
            x, y, w, h = roi
            Q[0, 3] += x * Q[0, 0]
            Q[1, 3] += y * Q[1, 1]
            disp = disp[y:y+h, x:x+w]
            rgb = rgb[y:y+h, x:x+w] if rgb is not None else None
        mask = disp > 0 if clip is None else \
            (disp > clip[0]) * (disp < clip[1])
        points = cv2.reprojectImageTo3D(disp, Q)[mask] * scale
        colors = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)[
            mask] if rgb is not None else None
        return PointCloud(points, colors)

    def save_ply(self, filename):
        with open(filename, 'wb') as f:
            f.writelines([
                b'ply\n',
                b'format ascii 1.0\n',
                b'element vertex %d\n' % len(self.points),
                b'property float x\n',
                b'property float y\n',
                b'property float z\n',
            ])
            if self.colors is not None:
                verts = np.hstack([self.points, self.colors])
                f.writelines([
                    b'property uchar red\n',
                    b'property uchar green\n',
                    b'property uchar blue\n',
                    b'end_header\n',
                ])
                np.savetxt(f, verts, fmt='%f %f %f %d %d %d')
            else:
                f.write(b'end_header\n')
                np.savetxt(f, self.points, fmt='%f %f %f')

    def save(self, filename, format='ply'):
        if format == 'ply':
            self.save_ply(filename)
