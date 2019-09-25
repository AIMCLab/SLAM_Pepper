#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "Client/QIClient.h"

QIClient::QIClient(const char* name)
    : m_name(name),
      m_shmem_size(0), m_shmem_ptr(nullptr)
{
}

QIClient::~QIClient()
{
    if (m_shmem_ptr)
        munmap(m_shmem_ptr, m_shmem_size);
}

void QIClient::openSharedMemory()
{
    const std::string tmpDir = "/tmp/NAOqiManager";
    int fd = open((tmpDir + '/' + m_name).c_str(), O_RDONLY);
    if (fd < 0)
    {
        puts("QIClient::openSharedMemory():\n\tShared memory open failed!");
        return;
    }

    m_shmem_size = lseek(fd, 0, SEEK_END);
    lseek(fd, 0L, SEEK_SET);
    void* ptr = mmap(nullptr, m_shmem_size, PROT_READ, MAP_SHARED, fd, 0);
    close(fd);

    if (ptr == MAP_FAILED)
        puts("QIClient::openSharedMemory():\n\tShared memory mmap failed!");
    else
        m_shmem_ptr = ptr;
}
