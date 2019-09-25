#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "Device/QIDevice.h"

QIDevice::QIDevice(boost::shared_ptr<AL::ALBroker> broker, const char* name)
    : m_broker(broker), m_name(name),
      m_shmem_size(0), m_shmem_ptr(nullptr)
{
}

QIDevice::~QIDevice()
{
    if (m_shmem_ptr)
        munmap(m_shmem_ptr, m_shmem_size);
}

void QIDevice::createSharedMemory(size_t size)
{
    const std::string tmpDir = "/tmp/NAOqiManager";
    m_shmem_size = size;
    mkdir(tmpDir.c_str(), 00777);
    int fd = open((tmpDir + '/' + m_name).c_str(), O_CREAT | O_RDWR | O_TRUNC, 00666);
    if (fd < 0)
    {
        throw AL::ALError(getName(), "createSharedMemory()", "Shared memory open failed!");
        return;
    }

    ftruncate(fd, size);
    void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    if (ptr == MAP_FAILED)
        throw AL::ALError(getName(), "createSharedMemory()", "Shared memory mmap failed!");
    else
        m_shmem_ptr = ptr;
}
