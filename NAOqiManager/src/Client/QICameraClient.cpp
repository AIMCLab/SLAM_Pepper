#include "Client/QICameraClient.h"

#include <boost/make_shared.hpp>

QICameraClient::QICameraClient(const char* name)
    : QIClient(name)
{
}

QICameraClient::~QICameraClient()
{
}

boost::shared_ptr<QIImage> QICameraClient::getImage() const
{
    if (!m_shmem_ptr)
    {
        puts("QICameraClient::getImage():\n\t Shared memory is not open");
        return boost::make_shared<QIImage>(0, 0, 0);
    }
    return boost::make_shared<QIImage>(m_shmem_ptr);
}
