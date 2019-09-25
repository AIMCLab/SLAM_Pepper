#include <boost/make_shared.hpp>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>

#include "Data/QIGazeAnalysisData.h"
#include "Data/QIImage.h"

using namespace boost::python;
namespace np = numpy;

boost::shared_ptr<QIImage> makeQIImage(int width, int height, int layers, int color_space, const list& data)
{
    int l = len(data);
    boost::shared_ptr<QIImage> obj;
    if (l > 0)
    {
        size_t size = QIImage::calcBufferSize(width, height, layers);
        uint8_t* buffer = new uint8_t[size];
        memset(buffer, 0, size);
        for (int i = 0; i < l && i < size; i++)
            buffer[i] = extract<uint8_t>(data[i]);
        obj = boost::make_shared<QIImage>(width, height, layers, color_space, buffer, true);
        delete[] buffer;
    }
    else
        obj = boost::make_shared<QIImage>(width, height, layers, color_space);
    return obj;
}

list getImageDataList(QIImage* self)
{
    list buffer;
    size_t size = self->getImageSize();
    uint8_t* ptr = (uint8_t*) self->getImageData();
    for (int i = 0; i < size; i++)
        buffer.append(*(ptr + i));
    return buffer;
}

np::ndarray getImageDataNdarray(QIImage* self)
{
    size_t s = sizeof(uint8_t);
    uint8_t* ptr = (uint8_t*) self->getImageData();
    int height = self->getHeight();
    int width = self->getWidth();
    int layers = self->getNbLayers();

    np::dtype dtype = np::dtype::get_builtin<boost::uint8_t>();
    auto shape = make_tuple(height, width, layers);
    auto stride = make_tuple(width * layers * s, layers * s, s);
    return np::from_data(ptr, dtype, shape, stride, object());
}

void expose_data()
{
    class_<QIData, boost::noncopyable>("QIData", no_init)
        .add_property("buffer_size", &QIData::getBufferSize)
        .def("dumpFile", &QIData::dumpFile);

    class_<QIImage, bases<QIData>>("QIImage", no_init)
        .def("__init__", make_constructor(makeQIImage, default_call_policies(),
                                          (arg("width"), arg("height"), arg("layers"),
                                           arg("color_space") = 0, arg("data") = list())))
        .add_property("image_size", &QIImage::getImageSize)
        .add_property("width", &QIImage::getWidth)
        .add_property("height", &QIImage::getHeight)
        .add_property("layers", &QIImage::getNbLayers)
        .add_property("color_space", &QIImage::getColorSpace)
        .add_property("camera_id", &QIImage::getCameraId)
        .add_property("time_stamp", &QIImage::getTimeStamp)
        .add_property("data", getImageDataList)
        .add_property("data_numpy", getImageDataNdarray)
        .def("calcBufferSize", &QIImage::calcBufferSize)
        .def("fromFile", &QIImage::fromFile);

    class_<QIGazeAnalysisData, bases<QIData>>("QIGazeAnalysisData", init<double, uint64_t>(args("looking_at_score",
                                                                                                "time_stamp")))
        .add_property("looking_at_score", &QIGazeAnalysisData::getLookingAtScore)
        .add_property("time_stamp", &QIGazeAnalysisData::getTimeStamp)
        .def("isLookingAt", &QIGazeAnalysisData::isLookingAt)
        .def("calcBufferSize", &QIGazeAnalysisData::calcBufferSize);

    register_ptr_to_python<boost::shared_ptr<QIImage>>();
    register_ptr_to_python<boost::shared_ptr<QIGazeAnalysisData>>();
}
