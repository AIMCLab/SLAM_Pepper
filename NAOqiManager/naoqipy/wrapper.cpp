#include <boost/python.hpp>
#include <boost/python/numpy.hpp>

void expose_data();
void expose_client();

BOOST_PYTHON_MODULE(libnaoqipy)
{
    boost::python::numpy::initialize();

    expose_data();
    expose_client();
}
