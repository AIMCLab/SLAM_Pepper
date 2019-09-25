#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "Client/QICameraClient.h"
#include "Client/QIGazeAnalysisClient.h"
#include "Data/QIImage.h"

const int FPS = 15;

int main(int argc, char* argv[])
{
    QICameraClient* stereoCamera = new QICameraClient("StereoCamera");
    QIGazeAnalysisClient* gazeAnalysiser = new QIGazeAnalysisClient("gazeAnalysiser");
    stereoCamera->openSharedMemory();
    gazeAnalysiser->openSharedMemory();

    while (true)
    {
        boost::shared_ptr<QIGazeAnalysisData> data = gazeAnalysiser->getData();
        printf("QIGazeAnalysisData %lf %lld %d\n", data->getLookingAtScore(), data->getTimeStamp(),
               data->isLookingAt());

        boost::shared_ptr<QIImage> image = stereoCamera->getImage();
        cv::Mat cvImg, tmp = cv::Mat(cv::Size(image->getWidth(), image->getHeight()), CV_8UC3);
        tmp.data = (unsigned char*) image->getImageData();
        cv::cvtColor(tmp, cvImg, CV_BGR2RGB);

        printf("QIImage %d %d %lld %d\n", image->getWidth(), image->getHeight(),
               image->getTimeStamp(), image->getColorSpace());
        cv::imshow("images", cvImg);
        cv::waitKey(1000 / FPS);
    }
    return 0;
}
