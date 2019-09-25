#ifndef QI_DEVICE_H
#define QI_DEVICE_H

#include <string>
#include <alcommon/albroker.h>
#include <alvalue/alvalue.h>

class QIDevice
{
public:
    QIDevice(boost::shared_ptr<AL::ALBroker> broker, const char* name);
    virtual ~QIDevice();

    virtual void pullRemoteData() = 0;
    virtual void writeSharedMemory() = 0;

    void createSharedMemory(size_t size);

    boost::shared_ptr<AL::ALBroker> getBroker() const { return m_broker; }
    std::string getName() const { return m_name; }

protected:
    boost::shared_ptr<AL::ALBroker> m_broker;
    std::string m_name;

    AL::ALValue m_remote_data;
    size_t m_shmem_size;
    void* m_shmem_ptr;
};

#endif // QI_DEVICE_H
