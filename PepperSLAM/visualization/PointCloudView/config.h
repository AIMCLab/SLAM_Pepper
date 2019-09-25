#ifndef CONFIG_H
#define CONFIG_H

class Config
{
public:
    static double fx, fy, cx, cy, fb;
    static double posX, posY, posZ;
    static double viewX, viewY, viewZ;
    static double upX, upY, upZ;
    static double minDisp, maxDisp, dispScale, visibleDisp;
    static double trajScale;

    static int pointSize;
    static double pointCloudRes;

    static void loadConfig(const char *fname)
    {
        cv::FileStorage fs(fname, cv::FileStorage::READ);
        assert(fs.isOpened());

        fx = fs["Camera.fx"];
        fy = fs["Camera.fy"];
        cx = fs["Camera.cx"];
        cy = fs["Camera.cy"];
        fb = fs["Camera.fb"];

        posX = fs["Viewer.posX"];
        posY = fs["Viewer.posY"];
        posZ = fs["Viewer.posZ"];
        viewX = fs["Viewer.viewX"];
        viewY = fs["Viewer.viewY"];
        viewZ = fs["Viewer.viewZ"];
        upX = fs["Viewer.upX"];
        upY = fs["Viewer.upY"];
        upZ = fs["Viewer.upZ"];

        minDisp = fs["Disparity.minDisp"];
        maxDisp = fs["Disparity.maxDisp"];
        visibleDisp = fs["Disparity.visibleDisp"];
        dispScale = fs["Disparity.scale"];

        pointSize = fs["PointCloud.PointSize"];
        pointCloudRes = fs["PointCloud.Resolution"];

        trajScale = fs["Trajectory.scale"];

        fs.release();
    }

private:
    Config();
};

double Config::fx, Config::fy, Config::cx, Config::cy, Config::fb;
double Config::posX, Config::posY, Config::posZ;
double Config::viewX, Config::viewY, Config::viewZ;
double Config::upX, Config::upY, Config::upZ;
double Config::minDisp, Config::maxDisp, Config::dispScale, Config::visibleDisp;
double Config::trajScale;

int Config::pointSize;
double Config::pointCloudRes;

#endif // CONFIG_H
