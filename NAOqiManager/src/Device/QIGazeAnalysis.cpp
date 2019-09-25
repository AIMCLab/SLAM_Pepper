#include <ctime>
#include <iostream>
#include <alproxies/algazeanalysisproxy.h>

#include "Device/QIGazeAnalysis.h"
#include "Device/QIMemory.h"

using namespace std;
using namespace AL;

QIGazeAnalysis::QIGazeAnalysis(boost::shared_ptr<ALBroker> broker, const char* name)
    : QIDevice(broker, name),
      m_gaze_analysis_proxy(broker ? new ALGazeAnalysisProxy(broker) : nullptr),
      m_qi_memory(broker ? new QIMemory(broker, name) : nullptr),
      m_gaze_analysis_data(0, 0), m_threshold(0.5)
{
}

QIGazeAnalysis::~QIGazeAnalysis()
{
    unregisterDevice();
}

void QIGazeAnalysis::pullRemoteData()
{
}

void QIGazeAnalysis::writeSharedMemory()
{
    if (!m_shmem_ptr)
    {
        throw ALError(getName(), "writeSharedMemory()", "Shared memory is not created!");
        return;
    }
    m_gaze_analysis_data.writeBuffer(m_shmem_ptr);
}

void QIGazeAnalysis::registerDevice()
{
    if (m_qi_memory == nullptr)
        return;
    m_qi_memory->subscribeEvent("GazeAnalysis/PersonStartsLookingAtRobot", [this](const ALValue& value) {
        int id = (int) value;
        cout << "Got HumanTracked start: detected person with ID: " << id << endl;
        if (id > 0)
        {
            string key = "PeoplePerception/Person/" + to_string(id) + "/LookingAtRobotScore";
            double score = m_qi_memory->getData(key);
            cout << "the tracked person starts looking at the robot: " << score << endl;
            if (score >= m_threshold)
                updataLookAt(score);
        }
    });
    m_qi_memory->subscribeEvent("GazeAnalysis/PersonStopsLookingAtRobot", [this](const ALValue& value) {
        int id = (int) value;
        cout << "Got HumanTracked stop: detected person with ID: " << id << endl;
        if (id > 0)
        {
            cout << "the tracked person stops looking at the robot" << endl;
            updataLookAt(0);
        }
    });
}

void QIGazeAnalysis::unregisterDevice()
{
    if (m_qi_memory)
        m_qi_memory->unsubscribeAll();
}

void QIGazeAnalysis::updataLookAt(double score)
{
    m_gaze_analysis_data = QIGazeAnalysisData(score, time(nullptr));
    writeSharedMemory();
}
