#ifndef QI_CAMERA_H
#define QI_CAMERA_H

#include "Device/QIDevice.h"

namespace AL
{
class ALVideoDeviceProxy;
}

class QICamera : public QIDevice
{
public:
    QICamera(boost::shared_ptr<AL::ALBroker> broker, const char* name);
    virtual ~QICamera();

    void pullRemoteData() override;
    void writeSharedMemory() override;

    void registerDevice(int cameraIndex, int resolution, int colorSpace, int fps);
    void unregisterDevice();

    void setParameter(int cameraIndex, int parameter, int newValue);

    QIImage getImage();

private:
    boost::shared_ptr<AL::ALVideoDeviceProxy> m_camera_proxy;
    std::string m_subscriber_id;
};

#endif // QI_CAMERA_H
