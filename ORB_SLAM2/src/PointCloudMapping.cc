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

#include "Converter.h"
#include "KeyFrame.h"
#include "PointCloudMapping.h"

#include <pcl/common/transforms.h>

namespace ORB_SLAM2
{

PointCloudMapping::PointCloudMapping(double resolution_)
    : mViewer(new pcl::visualization::PCLVisualizer("viewer"))
{
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);

    mViewer->setBackgroundColor(0, 0, 0);
    mViewer->addCoordinateSystem(0.5);

    viewerThread = make_shared<thread>(bind(&PointCloudMapping::Viewer, this));
}

void PointCloudMapping::Shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::InsertKeyFrame(KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);

    PointCloud::Ptr p = GeneratePointCloud(kf, color.clone(), depth.clone());
    char id[256];
    sprintf(id, "%lu", kf->mnId);

    mViewer->addPointCloud(p, id);
    mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id);

    keyFrameUpdated.notify_one();
}

void PointCloudMapping::EraseKeyFrame(KeyFrame* kf)
{
    // unique_lock<mutex> lck(keyframeMutex);
    // char id[256];
    // sprintf(id, "%lu", kf->mnId);
    // mViewer->removePointCloud(id);

    // keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::GeneratePointCloud(KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth)
{
    PointCloud::Ptr tmp(new PointCloud);
    // point cloud is null ptr

    for ( int m=0; m<depth.rows; m+=1 )
    {
        for ( int n=0; n<depth.cols; n+=1 )
        {
            float d = depth.at<float>(m, n);
            if (d < 0.01 || d>10)
                continue;
            PointT p;
            p.z = d;
            p.x = (n - kf->cx) * p.z / kf->fx;
            p.y = (m - kf->cy) * p.z / kf->fy;

            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(p);
        }
    }

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(kf->GetPose());
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud(*tmp, *cloud, T.inverse().matrix());
    tmp->swap(*cloud);
    voxel.setInputCloud(tmp);
    voxel.filter(*cloud);
    cloud->is_dense = false;

    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}


void PointCloudMapping::Viewer()
{
    while (!mViewer->wasStopped())
    {
        {
            unique_lock<mutex> lck_shutdown(shutDownMutex);
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
            keyFrameUpdated.wait(lck_keyframeUpdated);
        }
    }
}

} //namespace ORB_SLAM
