#ifndef QI_GAZE_ANALYSIS_DATA_H
#define QI_GAZE_ANALYSIS_DATA_H

#include "Data/QIData.h"

class QIGazeAnalysisData : public QIData
{
public:
    QIGazeAnalysisData(const AL::ALValue& value);
    QIGazeAnalysisData(double lookingAtScore, uint64_t timeStamp);
    QIGazeAnalysisData(const void* buffer);
    virtual ~QIGazeAnalysisData();

    virtual AL::ALValue toALValue() const override;
    virtual size_t getBufferSize() const override { return BUFFER_SIZE; }
    virtual void writeBuffer(void* dst) override;

    inline double getLookingAtScore() const { return m_looking_at_score; }
    inline uint64_t getTimeStamp() const { return m_time_stamp; }
    inline bool isLookingAt() const { return m_looking_at_score > 0; }

    static size_t calcBufferSize() { return BUFFER_SIZE; }

private:
    static constexpr size_t BUFFER_SIZE = sizeof(double) + sizeof(uint64_t);

    /// the confidence (between 0 and 1) in the fact that the person is looking at the robot.
    double m_looking_at_score;

    /// Time in microsecond when the people looking at the robot
    uint64_t m_time_stamp;
};

#endif // QI_GAZE_ANALYSIS_DATA_H
