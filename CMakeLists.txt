cmake_minimum_required(VERSION 3.0)

project(QCamControls)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-DQT_NO_KEYWORDS")

add_compile_definitions(HAVE_TIFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera) # libcamera-dev:amd64   0.r3948.ged591e70-1~ubuntu20.04

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(TIFF)

include_directories(
        .
        ${LIBCAMERA_INCLUDE_DIRS}
)

set(SOURCES
        main.cpp
        main_window.cpp main_window.h
        viewfinder_qt.cpp viewfinder_qt.h
        format_converter.cpp format_converter.h
        message_handler.cpp message_handler.h
        cam/options.cpp cam/options.h
        cam/stream_options.cpp cam/stream_options.h
        dng_writer.cpp dng_writer.h
        SensorConfig.cpp SensorConfig.h
        )

qt5_add_resources(SOURCES
        assets/feathericons/feathericons.qrc
        assets/shader/shaders.qrc
        )

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} PRIVATE
        Qt5::Widgets
        TIFF::TIFF
        ${LIBCAMERA_LIBRARIES}
        )
