#include "Data/QIGazeAnalysisData.h"

QIGazeAnalysisData::QIGazeAnalysisData(const AL::ALValue& value)
    : m_looking_at_score(value[0]),
      m_time_stamp(1000000ll * (int) value[1] + (int) value[2])
{
}

QIGazeAnalysisData::QIGazeAnalysisData(double lookingAtScore, uint64_t timeStamp)
    : m_looking_at_score(lookingAtScore), m_time_stamp(timeStamp)
{
}

QIGazeAnalysisData::QIGazeAnalysisData(const void* buffer)
{
    const char* ptr = (const char*) buffer;
    m_looking_at_score = *(double*) ptr, ptr += 8;
    m_time_stamp = *(uint64_t*) ptr, ptr += 8;
}

QIGazeAnalysisData::~QIGazeAnalysisData()
{
}

AL::ALValue QIGazeAnalysisData::toALValue() const
{
    AL::ALValue value;
    value[0] = m_looking_at_score;
    value[1] = (int) (m_time_stamp / 1000000);
    value[2] = (int) (m_time_stamp % 1000000);
    return value;
}

void QIGazeAnalysisData::writeBuffer(void* dst)
{
    char* ptr = (char*) dst;
    *(double*) ptr = m_looking_at_score, ptr += 8;
    *(uint64_t*) ptr = m_time_stamp, ptr += 8;
}
