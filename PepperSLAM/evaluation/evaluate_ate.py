#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Requirements:
# sudo apt-get install python-argparse

"""
This script computes the absolute trajectory error from the ground truth
trajectory and the estimated trajectory.
"""

import sys
import numpy as np
import argparse

from utils import trajectory

def plot_traj(ax,stamps,traj,style,color,label):
    """
    Plot a trajectory using matplotlib.

    Input:
    ax -- the plot
    stamps -- time stamps (nx1)
    traj -- trajectory (nx3)
    style -- line style
    color -- line color
    label -- plot legend

    """
    stamps.sort()
    interval = np.median([s-t for s,t in zip(stamps[1:],stamps[:-1])])
    x = []
    y = []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i]-last < 2*interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
        elif len(x)>0:
            ax.plot(x,y,style,color=color,label=label)
            label=""
            x=[]
            y=[]
        last= stamps[i]
    if len(x)>0:
        ax.plot(x,y,style,color=color,label=label)

def find_best_scale(points1, points2, l, r):
    def f(scale):
        rot, trans, trans_error = trajectory.align_trajectories(points1, points2 * scale)
        return np.sqrt(np.dot(trans_error, trans_error) / len(trans_error))

    while r - l > 1e-6:
        ll, rr = (r - l) / 3.0 + l, 2.0 * (r - l) / 3.0 + l
        if f(ll) - f(rr) > 1e-6:
            l = ll
        else:
            r = rr
    return l, f(l)


if __name__=="__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory.
    ''')
    parser.add_argument('first_file', help='ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('second_file', help='estimated trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--scale', help='scaling factor for the second trajectory (default: 1.0)',default=1.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)
    parser.add_argument('--save', help='save aligned second trajectory to disk (format: stamp2 x2 y2 z2)')
    parser.add_argument('--save_associations', help='save associated first and aligned second trajectory to disk (format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)')
    parser.add_argument('--plot', help='plot the first and the aligned second trajectory to an image (format: png)')
    parser.add_argument('--verbose', '-v', help='print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)', action='store_true')
    parser.add_argument('--find_scale', '-f', help='automatically find the best scale', action='store_true')
    args = parser.parse_args()

    first_list = trajectory.read_file_list(args.first_file)
    second_list = trajectory.read_file_list(args.second_file)

    matches = trajectory.associate(first_list, second_list,float(args.offset),float(args.max_difference))
    if len(matches)<2:
        sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")

    first_xyz = np.array([[float(value) for value in first_list[a][0:3]] for a,b in matches])
    second_xyz = np.array([[float(value) for value in second_list[b][0:3]] for a,b in matches])

    if args.find_scale:
        scale, _ = find_best_scale(first_xyz, second_xyz, 0, 2)
        print('Best scale: %f' % scale)
    else:
        scale = args.scale
    second_xyz *= scale
    rot, trans, trans_error = trajectory.align_trajectories(first_xyz, second_xyz)

    second_xyz_aligned = (rot.dot(second_xyz.T) + trans).T

    first_stamps = first_list.keys()
    first_stamps.sort()
    first_xyz_full = np.array([[float(value) for value in first_list[b][0:3]] for b in first_stamps])

    second_stamps = second_list.keys()
    second_stamps.sort()
    second_xyz_full = np.array([[float(value)*scale for value in second_list[b][0:3]] for b in second_stamps])
    second_xyz_full_aligned = (rot.dot(second_xyz_full.T) + trans).T

    if args.verbose:
        print("compared_pose_pairs %d pairs"%(len(trans_error)))

        print("absolute_translational_error.rmse %f m"%np.sqrt(np.dot(trans_error,trans_error) / len(trans_error)))
        print("absolute_translational_error.mean %f m"%np.mean(trans_error))
        print("absolute_translational_error.median %f m"%np.median(trans_error))
        print("absolute_translational_error.std %f m"%np.std(trans_error))
        print("absolute_translational_error.min %f m"%np.min(trans_error))
        print("absolute_translational_error.max %f m"%np.max(trans_error))
    else:
        print("%f"%np.sqrt(np.dot(trans_error,trans_error) / len(trans_error)))

    if args.save_associations:
        file = open(args.save_associations,"w")
        file.write("\n".join(["%f %f %f %f %f %f %f %f"%(a,x1,y1,z1,b,x2,y2,z2) for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz, second_xyz_aligned)]))
        file.close()

    if args.save:
        file = open(args.save,"w")
        file.write("\n".join(["%f "%stamp+" ".join(["%f"%d for d in line]) for stamp,line in zip(second_stamps, second_xyz_full_aligned)]))
        file.close()

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.pylab as pylab
        from matplotlib.patches import Ellipse
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plot_traj(ax,first_stamps,first_xyz_full,'-',"black","ground truth")
        plot_traj(ax,second_stamps,second_xyz_full_aligned,'-',"blue","estimated")

        label="difference"
        for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz,second_xyz_aligned):
            ax.plot([x1,x2],[y1,y2],'-',color="red",label=label)
            label=""

        ax.legend()

        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        plt.savefig(args.plot,dpi=90)

