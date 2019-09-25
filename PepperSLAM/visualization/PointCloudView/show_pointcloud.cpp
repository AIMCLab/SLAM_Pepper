#include <cstdlib>
#include <iostream>
#include <opencv/cv.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>

#include "config.h"

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBNormal PointNormalT;
typedef pcl::PointCloud<PointT> PointCloud;

cv::Mat read_data(const string &fname, int width, int height)
{
    FILE *fin = fopen(fname.c_str(), "rb");
    cv::Mat d(cv::Size(width, height), CV_16UC1);
    fread(d.data, 2, width * height, fin);
    d.convertTo(d, CV_32F, Config::maxDisp / Config::dispScale);
    fclose(fin);
    return d;
}

PointCloud::Ptr generate_point_cloud(const cv::Mat &rgb, const cv::Mat &disp)
{
    PointCloud::Ptr pc(new PointCloud);
    for (int i = 0; i < disp.rows; i++)
        for (int j = 0; j < disp.cols; j++)
        {
            if (j < 20 || j + 20 >= disp.cols)
                continue;

            double d = disp.at<float>(i, j);
            if (d <= 0 || d < Config::minDisp)
                continue;

            PointT p;
            p.z = Config::fb / d;
            p.x = (j - Config::cx) * p.z / Config::fx;
            p.y = (i - Config::cy) * p.z / Config::fy;

            p.b = rgb.at<cv::Vec3b>(i, j + 10)[0];
            p.g = rgb.at<cv::Vec3b>(i, j + 10)[1];
            p.r = rgb.at<cv::Vec3b>(i, j + 10)[2];

            pc->points.push_back(p);
        }
    return pc;
}

PointCloud::Ptr pre_filter(PointCloud::Ptr cloud)
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(200);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);
    cout << cloud->size() << endl;

    pcl::VoxelGrid<PointT> voxel;
    // voxel.setLeafSize(0.05, 0.05, 0.05);
    // voxel.setInputCloud(cloud);
    // voxel.filter(*cloud);
    // cout << cloud->size() << endl;

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
    // mls.setPolynomialFit(false);
    mls.setSearchRadius(0.5);
    mls.process(*output);

    cout << output->size() << endl;

    return output;
}

pcl::PointCloud<PointNormalT>::Ptr get_normals(PointCloud::Ptr cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    pcl::NormalEstimation<PointT, pcl::Normal> n;
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);

    n.setKSearch(20);
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
    gp3.setMu(5);
    gp3.setMaximumNearestNeighbors(200);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3.setNormalConsistency(false);
    gp3.reconstruct(mesh);

    return mesh;
}

int main(int argc, char *argv[])
{
    const char *rgb_file = argv[1];
    const char *disp_file = argv[2];
    const char *config_file = argv[3];
    Config::loadConfig(config_file);
    cv::Mat rgb = cv::imread(rgb_file);
    cv::Mat disp = read_data(disp_file, rgb.cols - 40, rgb.rows);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
    // viewer->setBackgroundColor(1, 1, 1);
    viewer->setShowFPS(false);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setSize(900, 900);
    viewer->setCameraPosition(Config::posX, Config::posY, Config::posZ,
                              Config::viewX, Config::viewY, Config::viewZ,
                              Config::upX, Config::upY, Config::upZ);

    PointCloud::Ptr cloud = generate_point_cloud(rgb, disp);
    cout << cloud->size() << endl;

    cloud = pre_filter(cloud);
    PointCloud::Ptr smooth_cloud = smooth(cloud);

    pcl::PointCloud<PointNormalT>::Ptr cloud_with_normals = get_normals(smooth_cloud);
    pcl::PolygonMesh mesh = reconstruct_mesh(cloud_with_normals);
    pcl::io::savePLYFile("mesh.ply", mesh);

    viewer->addPointCloud(cloud, "cloud");
    viewer->addPolygonMesh(mesh, "mesh");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, Config::pointSize, "cloud");

    viewer->spinOnce(100);
    viewer->saveScreenshot("a.png");

    std::vector<pcl::visualization::Camera> cameras;
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        // viewer->getCameras(cameras);
        // cout << "POS " << cameras[0].pos[0] << ' ' << cameras[0].pos[1] << ' ' << cameras[0].pos[2] << endl;
        // cout << "VIEW " << cameras[0].focal[0] << ' ' << cameras[0].focal[1] << ' ' << cameras[0].focal[2] << endl;
        // cout << "UP " << cameras[0].view[0] << ' ' << cameras[0].view[1] << ' ' << cameras[0].view[2] << endl;
    }

    return 0;
}
