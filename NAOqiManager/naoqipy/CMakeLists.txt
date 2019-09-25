find_package(PythonLibs 3 REQUIRED)

if(APPLE)
set(CMAKE_SHARED_LIBRARY_SUFFIX ".so")
endif(APPLE)

include_directories(${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

aux_source_directory(. SRCS)

add_library(naoqipy SHARED ${SRCS})
target_link_libraries(naoqipy qidata qiclient)
target_link_libraries(naoqipy ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})
