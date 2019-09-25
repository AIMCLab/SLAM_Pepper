#include <cstdlib>
#include <iostream>

#include <opencv/cv.hpp>
#include <Eigen/Geometry>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <thread>

#include "config.h"

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointXYZT;
typedef pcl::PointXYZRGBNormal PointNormalT;
typedef pcl::PointCloud<PointT> PointCloud;

cv::Mat read_data(const string &fname, int width, int height)
{
    FILE *fin = fopen(fname.c_str(), "rb");
    cv::Mat d(cv::Size(width, height), CV_16UC1);
    fread(d.data, 2, width * height, fin);
    d.convertTo(d, CV_32F, Config::maxDisp / 65536.0);
    fclose(fin);
    return d;
}

PointCloud::Ptr stat_outlier_filter(PointCloud::Ptr cloud)
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(100);
    sor.setStddevMulThresh(2);
    sor.filter(*cloud);
    cout << cloud->size() << endl;

    return cloud;
}

PointCloud::Ptr radius_outlier_filter(PointCloud::Ptr cloud)
{
    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(0.3);
    ror.setMinNeighborsInRadius(200);
    ror.filter(*cloud);
    cout << cloud->size() << endl;

    return cloud;
}

PointCloud::Ptr voxel_filter(PointCloud::Ptr cloud)
{
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(Config::pointCloudRes, Config::pointCloudRes, Config::pointCloudRes);
    voxel.setInputCloud(cloud);
    voxel.filter(*cloud);
    cout << cloud->size() << endl;

    return cloud;
}

PointCloud::Ptr smooth(PointCloud::Ptr cloud)
{
    pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    pcl::MovingLeastSquares<PointT, PointT> mls;
    mls.setInputCloud(cloud);
    mls.setSearchMethod(tree);

    mls.setComputeNormals(false);
    mls.setPolynomialOrder(1);
    mls.setPolynomialFit(false);
    mls.setSearchRadius(0.3);
    mls.process(*output);

    cout << output->size() << endl;

    return output;
}

PointCloud::Ptr pass_through(PointCloud::Ptr cloud)
{
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2.3, 2.3);
    pass.filter(*cloud);

    pass.setFilterFieldName("x");
    pass.setFilterLimits(-2.5, 2.5);
    pass.filter(*cloud);

    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2.5, 6);
    pass.filter(*cloud);

    cout << cloud->size() << endl;

    return cloud;
}

pcl::PointCloud<PointNormalT>::Ptr get_normals(PointCloud::Ptr cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    pcl::NormalEstimation<PointT, pcl::Normal> n;
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);

    n.setKSearch(500);
    n.compute(*normals);

    pcl::PointCloud<PointNormalT>::Ptr cloud_with_normals(new pcl::PointCloud<PointNormalT>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    return cloud_with_normals;
}

pcl::PolygonMesh reconstruct_mesh(pcl::PointCloud<PointNormalT>::Ptr cloud)
{
    pcl::PolygonMesh mesh;
    pcl::search::KdTree<PointNormalT>::Ptr tree(new pcl::search::KdTree<PointNormalT>);
    tree->setInputCloud(cloud);

    pcl::GreedyProjectionTriangulation<PointNormalT> gp3;
    gp3.setInputCloud(cloud);
    gp3.setSearchMethod(tree);

    gp3.setSearchRadius(1);
    gp3.setMu(3);
    gp3.setMaximumNearestNeighbors(500);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3.setNormalConsistency(false);
    gp3.reconstruct(mesh);

    return mesh;
}

int main(int argc, char *argv[])
{
    Config::loadConfig(argv[2]);
    const char *pcd_file = argv[1];

    PointCloud::Ptr cloud(new PointCloud);
    pcl::io::loadPCDFile<PointT>(pcd_file, *cloud);
    cout << cloud->size() << endl;

    cloud = pass_through(cloud);
    pcl::io::savePCDFileBinary("map_pass.pcd", *cloud);

    // cloud = voxel_filter(cloud);

    // pcl::io::savePCDFileBinary("map_voxel.pcd", *cloud);
    // cloud = stat_outlier_filter(cloud);
    cloud = radius_outlier_filter(cloud);
    pcl::io::savePCDFileBinary("map_outlier.pcd", *cloud);

    cloud = smooth(cloud);
    pcl::io::savePCDFileBinary("map_smooth.pcd", *cloud);

    // pcl::PointCloud<PointNormalT>::Ptr cloud_with_normals = get_normals(cloud);
    // cout<<"normal "<<cloud_with_normals->size()<<endl;
    // pcl::PolygonMesh mesh = reconstruct_mesh(cloud_with_normals);
    // pcl::io::savePLYFile("mesh.ply", mesh);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
    // viewer->setBackgroundColor(1, 1, 1);
    viewer->setShowFPS(false);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    // viewer->setSize(900, 900);
    viewer->setCameraPosition(Config::posX, Config::posY, Config::posZ,
                              Config::viewX, Config::viewY, Config::viewZ,
                              Config::upX, Config::upY, Config::upZ);

    viewer->addPointCloud(cloud, "cloud");
    // viewer->addPolygonMesh(mesh, "mesh");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, Config::pointSize, "cloud");

    // viewer->spin();

    viewer->saveScreenshot("a.png");

    std::vector<pcl::visualization::Camera> cameras;
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        viewer->getCameras(cameras);
        cout << "POS " << cameras[0].pos[0] << ' ' << cameras[0].pos[1] << ' ' << cameras[0].pos[2] << endl;
        cout << "VIEW " << cameras[0].focal[0] << ' ' << cameras[0].focal[1] << ' ' << cameras[0].focal[2] << endl;
        cout << "UP " << cameras[0].view[0] << ' ' << cameras[0].view[1] << ' ' << cameras[0].view[2] << endl;
    }

    return 0;
}
