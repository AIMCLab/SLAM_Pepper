/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "KeyFrame.h"

#include <thread>
#include <condition_variable>

#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace ORB_SLAM2
{

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloudMapping(double resolution_);

    void InsertKeyFrame(KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth);
    void EraseKeyFrame(KeyFrame* kf);
    void Shutdown();

private:
    void Viewer();

    PointCloud::Ptr GeneratePointCloud(KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth);

    pcl::visualization::PCLVisualizer::Ptr mViewer;
    shared_ptr<thread> viewerThread;

    bool shutDownFlag = false;
    mutex shutDownMutex;

    condition_variable keyFrameUpdated;
    mutex keyFrameUpdateMutex;
    mutex keyframeMutex;

    double resolution = 0.04;
    pcl::VoxelGrid<PointT>  voxel;
};

} //namespace ORB_SLAM

#endif // POINTCLOUDMAPPING_H
