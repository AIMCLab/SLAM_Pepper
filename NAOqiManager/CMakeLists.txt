cmake_minimum_required(VERSION 3.0)

project(NAOqiManager)
set(CMAKE_CXX_STANDARD 11)

find_package(Boost 1.63 REQUIRED COMPONENTS python3 numpy3)
find_package(qibuild REQUIRED)

include_directories(src)

aux_source_directory("src/Data" DATA_SRCS)
aux_source_directory("src/Client" CLIENT_SRCS)
aux_source_directory("src/Device" DEVICE_SRCS)

qi_create_lib(qidata ${DATA_SRCS} DEPENDS ALVALUE)

qi_create_lib(qiclient ${CLIENT_SRCS})
target_link_libraries(qiclient qidata)

qi_create_lib(qidevice ${DEVICE_SRCS} DEPENDS ALVISION)
target_link_libraries(qidevice qidata)

qi_create_bin(main "src/bin/main.cpp")
target_link_libraries(main qidata qidevice)

qi_create_bin(client "src/bin/client.cpp" DEPENDS OPENCV2_IMGPROC OPENCV2_CORE OPENCV2_HIGHGUI)
target_link_libraries(client qidata qiclient)

add_subdirectory(naoqipy)
