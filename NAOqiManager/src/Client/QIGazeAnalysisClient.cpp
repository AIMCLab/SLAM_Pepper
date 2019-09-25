#include "Client/QIGazeAnalysisClient.h"

#include <boost/make_shared.hpp>

QIGazeAnalysisClient::QIGazeAnalysisClient(const char* name)
    : QIClient(name)
{
}

QIGazeAnalysisClient::~QIGazeAnalysisClient()
{
}

boost::shared_ptr<QIGazeAnalysisData> QIGazeAnalysisClient::getData() const
{
    if (!m_shmem_ptr)
    {
        puts("QIGazeAnalysisClient::getData():\n\t Shared memory is not open");
        return boost::make_shared<QIGazeAnalysisData>(0, 0);
    }
    return boost::make_shared<QIGazeAnalysisData>(m_shmem_ptr);
}
