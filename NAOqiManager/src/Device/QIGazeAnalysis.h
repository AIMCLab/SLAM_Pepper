#ifndef QI_GAZE_ANALYSIS_H
#define QI_GAZE_ANALYSIS_H

#include "Data/QIGazeAnalysisData.h"
#include "Device/QIDevice.h"

namespace AL
{
class ALGazeAnalysisProxy;
}

class QIMemory;

class QIGazeAnalysis : public QIDevice
{
public:
    QIGazeAnalysis(boost::shared_ptr<AL::ALBroker> broker, const char* name);
    virtual ~QIGazeAnalysis();

    void pullRemoteData() override;
    void writeSharedMemory() override;

    void registerDevice();
    void unregisterDevice();

    double getThreshold() const { return m_threshold; }
    void setThreshold(double t) { m_threshold = t; }

private:
    boost::shared_ptr<AL::ALGazeAnalysisProxy> m_gaze_analysis_proxy;
    boost::shared_ptr<QIMemory> m_qi_memory;
    QIGazeAnalysisData m_gaze_analysis_data;
    double m_threshold;

    void updataLookAt(double score);
};

#endif // QI_GAZE_ANALYSIS_H
