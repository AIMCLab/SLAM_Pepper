#include <iostream>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>

#include "Data/QIImage.h"
#include "Device/QICamera.h"

using namespace std;
using namespace AL;

QICamera::QICamera(boost::shared_ptr<ALBroker> broker, const char* name)
    : QIDevice(broker, name),
      m_camera_proxy(broker ? new ALVideoDeviceProxy(broker) : nullptr)
{
}

QICamera::~QICamera()
{
    unregisterDevice();
}

void QICamera::registerDevice(int cameraIndex, int resolution, int colorSpace, int fps)
{
    if (m_camera_proxy)
        m_subscriber_id = m_camera_proxy->subscribeCamera(m_name, cameraIndex, resolution, colorSpace, fps);
}

void QICamera::unregisterDevice()
{
    if (m_camera_proxy)
        m_camera_proxy->unsubscribe(m_subscriber_id);
    m_subscriber_id = "";
}

void QICamera::setParameter(int cameraIndex, int parameter, int newValue)
{
    if (m_camera_proxy)
        m_camera_proxy->setParameter(cameraIndex, parameter, newValue);
}

void QICamera::pullRemoteData()
{
    getImage();
}

void QICamera::writeSharedMemory()
{
    if (!m_shmem_ptr)
    {
        throw ALError(getName(), "writeSharedMemory()", "Shared memory is not created!");
        return;
    }
    if (!m_remote_data.isValid())
    {
        throw ALError(getName(), "writeSharedMemory()", "No remote data!");
        return;
    }

    QIImage image(m_remote_data);
    cout << image.getWidth() << ' ' << image.getHeight() << ' ' << image.getTimeStamp() << ' ' << image.getColorSpace() << endl;

    image.writeBuffer(m_shmem_ptr);
}

QIImage QICamera::getImage()
{
    if (m_subscriber_id != "")
        m_remote_data = m_camera_proxy->getImageRemote(m_subscriber_id);
    else
    {
        // throw ALError(getName(), "pullRemoteData()", "Device unregistered!");
        char data[1024 * 1024];
        FILE* f = fopen("a.data", "rb");
        fread(data, 1, 1024 * 1024, f);
        fclose(f);
        m_remote_data = QIImage(data).toALValue();
        m_remote_data[3] = rand();
    }
    return QIImage(m_remote_data);
}
