#ifndef QI_GAZE_ANALYSIS_CLIENT_H
#define QI_GAZE_ANALYSIS_CLIENT_H

#include "Client/QIClient.h"
#include "Data/QIGazeAnalysisData.h"

#include <boost/shared_ptr.hpp>

class QIGazeAnalysisClient : public QIClient
{
public:
    QIGazeAnalysisClient(const char* name);
    ~QIGazeAnalysisClient();

    boost::shared_ptr<QIGazeAnalysisData> getData() const;

private:
};

#endif // QI_GAZE_ANALYSIS_CLIENT_H
