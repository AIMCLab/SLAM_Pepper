#include <cstdlib>
#include <iostream>

#include <opencv/cv.hpp>
#include <Eigen/Geometry>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

#include <thread>

#include "config.h"

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointXYZT;
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

PointCloud::Ptr outlier_filter(PointCloud::Ptr cloud)
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(200);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);
    cout << cloud->size() << endl;

    return cloud;
}

PointCloud::Ptr voxel_filter(PointCloud::Ptr cloud)
{
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(Config::pointCloudRes, Config::pointCloudRes, Config::pointCloudRes);
    voxel.setInputCloud(cloud);
    voxel.setDownsampleAllData(true);
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
    // mls.setPolynomialFit(false);
    mls.setSearchRadius(0.5);
    mls.process(*output);

    cout << output->size() << endl;

    return output;
}

PointCloud::Ptr generate_point_clound(const cv::Mat &rgb, const cv::Mat &disp, const Eigen::Isometry3d &T)
{
    PointCloud::Ptr cloud(new PointCloud);
    for (int i = 0; i < disp.rows; i++)
        for (int j = 0; j < disp.cols; j++)
        {
            if (j < 20 || j + 20 >= disp.cols)
                continue;

            double d = disp.at<float>(i, j);
            if (d <= 0 || d < Config::minDisp || d > Config::visibleDisp)
                continue;

            Eigen::Vector3d point;
            point[2] = Config::fb / d;
            point[0] = (j - Config::cx) * point[2] / Config::fx;
            point[1] = (i - Config::cy) * point[2] / Config::fy;
            Eigen::Vector3d pointWorld = T * point;

            PointT p;
            p.x = pointWorld[0];
            p.y = pointWorld[1];
            p.z = pointWorld[2];
            p.b = rgb.at<cv::Vec3b>(i, j + 10)[0];
            p.g = rgb.at<cv::Vec3b>(i, j + 10)[1];
            p.r = rgb.at<cv::Vec3b>(i, j + 10)[2];

            cloud->points.push_back(p);
        }

    cout << cloud->size() << endl;
    cloud = outlier_filter(cloud);
    // cloud = smooth(cloud);
    // cloud = voxel_filter(cloud);

    return cloud;
}

PointCloud::Ptr pass_through(PointCloud::Ptr cloud)
{
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-5, 2.3);
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

void view(pcl::visualization::PCLVisualizer::Ptr viewer, char *argv[])
{
    const char *rgb_dir = argv[1];
    const char *disp_dir = argv[2];
    const char *traj_file = argv[3];
    const char *kf_traj_file = argv[4];
    const char *config_file = argv[5];

    map<double, int> frame_map;
    map<double, double> X, Y, Z, xx, yy, zz;
    ifstream fin(traj_file);
    for (int i = 0; !fin.eof(); i++)
    {
        double time;
        fin >> time;
        frame_map[time] = i + 7;
        fin >> X[time];
        fin >> Y[time];
        fin >> Z[time];
        fin >> xx[time];
        fin >> yy[time];
        fin >> zz[time];
        // for (int i = 0; i < 3; i++)
        //     fin >> time;
    }
    fin.close();

    char buf[256];
    double old_x, old_y, old_z;
    PointXYZT old_pos, pos;
    PointCloud::Ptr global(new PointCloud);

    fin.open(kf_traj_file);
    for (int i = 0; !fin.eof(); i++)
    {
        double time, x, y, z, w;
        fin >> time;
        fin >> pos.x >> pos.y >> pos.z >> x >> y >> z >> w;
        // pos.x = X[time];
        // pos.y = Y[time];
        // pos.z = Z[time];
        // x = xx[time];
        // y = yy[time];
        // z = zz[time];
        pos.x *= Config::trajScale;
        pos.y *= Config::trajScale;
        pos.z *= Config::trajScale;

        if (fin.eof())
            break;
        if (frame_map.find(time) == frame_map.end())
        {
            cout << "Keyframe at time " << time << " not found in " << traj_file << "!" << endl;
            continue;
        }
        int id = frame_map[time];
        cout << i << ' ' << id << ' ' << time << endl;

        // using namespace Eigen;
        // Matrix3d m;
        // m = AngleAxisd(x, Vector3d::UnitZ()) * AngleAxisd(-z + 0.6049430966377258, Vector3d::UnitY()) * AngleAxisd(y, Vector3d::UnitX());
        // Eigen::Quaterniond q(m);
        // cout << data[3] << ' ' << data[4] << ' ' << data[5] << endl;
        // cout << m << endl;
        Eigen::Quaterniond q(w, x, y, z);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(pos.x, pos.y, pos.z));
        if (i > 0)
        {
            sprintf(buf, "line%d", id);
            viewer->addLine(old_pos, pos, 1, 0, 0, string(buf));

            Eigen::Vector3d a(0.1, -0.05, 0), b(0.1, 0.05, 0), c(-0.1, 0.05, 0), d(-0.1, -0.05, 0);
            a = T * a;
            b = T * b;
            c = T * c;
            d = T * d;

            viewer->addLine(PointXYZT(a[0], a[1], a[2]), PointXYZT(b[0], b[1], b[2]), 0, 1, 0, string(buf) + "_a");
            viewer->addLine(PointXYZT(b[0], b[1], b[2]), PointXYZT(c[0], c[1], c[2]), 0, 1, 0, string(buf) + "_b");
            viewer->addLine(PointXYZT(c[0], c[1], c[2]), PointXYZT(d[0], d[1], d[2]), 0, 1, 0, string(buf) + "_c");
            viewer->addLine(PointXYZT(d[0], d[1], d[2]), PointXYZT(a[0], a[1], a[2]), 0, 1, 0, string(buf) + "_d");
            viewer->addLine(PointXYZT(a[0], a[1], a[2]), PointXYZT(c[0], c[1], c[2]), 0, 1, 0, string(buf) + "_e");
            viewer->addLine(PointXYZT(b[0], b[1], b[2]), PointXYZT(d[0], d[1], d[2]), 0, 1, 0, string(buf) + "_f");
            // Eigen::Vector3d pointWorld = T * point;
            // *rect += PointXYZT(1, 0, 0);
            // *rect += PointXYZT(1, 1, 0);
            // *rect += PointXYZT(-1, 1, 0);
            // *rect += PointXYZT(-1, -1, 0);
            // *rect += PointXYZT(-1, -1, 0);
        }
        old_pos = pos;

        sprintf(buf, "%s/%d.png", rgb_dir, id);
        cout << buf << endl;
        cv::Mat rgb = cv::imread(buf);
        sprintf(buf, "%s/%d.data", disp_dir, id);
        cout << buf << endl;
        cv::Mat disp = read_data(buf, rgb.cols - 40, rgb.rows);

        sprintf(buf, "frame%d", id);

        // PointCloud::Ptr cloud = generate_point_clound(rgb, disp, T);
        // *global += *cloud;
        // viewer->addPointCloud(cloud, buf);
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, Config::pointSize, buf);

        // viewer->spinOnce(10);
        usleep(100000);
    }
    fin.close();

    // global = voxel_filter(global);
    // global =
    // pcl::io::savePCDFileBinary("map.pcd", *global);

    cout << "OK" << endl;
}

int main(int argc, char *argv[])
{
    Config::loadConfig(argv[5]);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->setBackgroundColor(1, 1, 1);
    viewer->setShowFPS(false);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    // viewer->setSize(900, 900);
    viewer->setCameraPosition(Config::posX, Config::posY, Config::posZ,
                              Config::viewX, Config::viewY, Config::viewZ,
                              Config::upX, Config::upY, Config::upZ);

    PointCloud::Ptr cloud(new PointCloud);
    pcl::io::loadPCDFile<PointT>("semantic/map_smooth.pcd", *cloud);
    // pcl::io::loadPCDFile<PointT>("rgb/map_smooth.pcd", *cloud);
    cloud = pass_through(cloud);

    viewer->addPointCloud(cloud, "2333");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, Config::pointSize, "2333");

    // viewer->spinOnce(100);
    // thread viewerThread(bind(&view, viewer, argv));

    view(viewer, argv);
    // view(viewer, argv);

    // viewer->spin();
    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce(10);
    // }

    viewer->spinOnce(100);
    std::vector<pcl::visualization::Camera> cameras;
    viewer->saveScreenshot("a.png");
    int n = 0;
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        viewer->getCameras(cameras);
        cout << "POS " << cameras[0].pos[0] << ' ' << cameras[0].pos[1] << ' ' << cameras[0].pos[2] << endl;
        cout << "VIEW " << cameras[0].focal[0] << ' ' << cameras[0].focal[1] << ' ' << cameras[0].focal[2] << endl;
        cout << "UP " << cameras[0].view[0] << ' ' << cameras[0].view[1] << ' ' << cameras[0].view[2] << endl;
        if (++n < 20)
            viewer->saveScreenshot("a.png");
    }

    return 0;
}
