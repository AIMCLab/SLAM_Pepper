#include <alcommon/albroker.h>
#include <alvision/alvisiondefinitions.h>
#include <unistd.h>

#include "Data/QIImage.h"
#include "Device/QICamera.h"
#include "Device/QIGazeAnalysis.h"

const int PORT = 9559;
const int FPS = 15;

int main(int argc, char* argv[])
{
    boost::shared_ptr<AL::ALBroker> broker = nullptr;
    if (argc > 1)
    {
        char* ip = argv[1];
        printf("%s:%d\n", ip, PORT);
        broker = AL::ALBroker::createBroker("MyBroker", "", 0, ip, PORT);
    }

    QICamera* stereoCamera = new QICamera(broker, "StereoCamera");
    stereoCamera->registerDevice(AL::kStereoCamera, AL::kQQ720px2, AL::kRGBColorSpace, FPS);
    stereoCamera->setParameter(AL::kStereoCamera, AL::kCameraBrightnessID, 6);
    stereoCamera->createSharedMemory(QIImage::calcBufferSize(2560, 720, 3));

    QIGazeAnalysis* gazeAnalysiser = new QIGazeAnalysis(broker, "gazeAnalysiser");
    gazeAnalysiser->createSharedMemory(QIGazeAnalysisData::calcBufferSize());
    gazeAnalysiser->registerDevice();

    while (true)
    {
        stereoCamera->pullRemoteData();

        stereoCamera->writeSharedMemory();

        usleep(1000000 / FPS);
    }
    return 0;
}
