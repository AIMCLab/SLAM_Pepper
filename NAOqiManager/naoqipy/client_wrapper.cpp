#include <boost/python.hpp>

#include "Client/QICameraClient.h"
#include "Client/QIGazeAnalysisClient.h"

using namespace boost::python;

void expose_client()
{
    class_<QIClient, boost::noncopyable>("QIClient", no_init)
        .add_property("name", &QIClient::getName)
        .def("openSharedMemory", &QIClient::openSharedMemory);

    class_<QICameraClient, bases<QIClient>>("QICameraClient", init<const char*>(args("name")))
        .def("getImage", &QICameraClient::getImage);

    class_<QIGazeAnalysisClient, bases<QIClient>>("QIGazeAnalysisClient", init<const char*>(args("name")))
        .def("getData", &QIGazeAnalysisClient::getData);
}
