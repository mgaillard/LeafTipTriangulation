if(TARGET opencv::opencv)
    return()
endif()

message(STATUS "Third-party (external): creating target 'opencv'")

option(WITH_CUDA OFF)
option(WITH_LAPACK OFF)
option(WITH_OPENMP ON)
option(WITH_VTK OFF)

option(BUILD_JAVA OFF)

option(BUILD_opencv_java_bindings_generator OFF)
option(BUILD_opencv_js OFF)
option(BUILD_opencv_js_bindings_generator OFF)
option(BUILD_opencv_objc_bindings_generator OFF)
option(BUILD_opencv_python3 OFF)
option(BUILD_opencv_python_bindings_generator OFF)
option(BUILD_opencv_python_tests OFF)
option(BUILD_opencv_ts OFF)
option(BUILD_opencv_apps OFF)
option(BUILD_opencv_dnn OFF)
option(BUILD_opencv_gapi OFF)
option(BUILD_opencv_objdetect OFF)
option(BUILD_opencv_photo OFF)
option(BUILD_opencv_stitching OFF)
option(BUILD_opencv_world OFF)

# option(BUILD_opencv_core ON)
# option(BUILD_opencv_imgcodecs ON)
# option(BUILD_opencv_imgproc ON)
# option(BUILD_opencv_highgui ON)
# option(BUILD_opencv_calib3d ON)
# option(BUILD_opencv_features2d ON)
# option(BUILD_opencv_flann ON)
# option(BUILD_opencv_aruco ON)
# option(BUILD_opencv_ml ON)
# option(BUILD_opencv_video ON)
# option(BUILD_opencv_videoio ON)

include(FetchContent)
FetchContent_Declare(opencv
    GIT_REPOSITORY https://github.com/opencv/opencv.git
    GIT_TAG        4.5.5
    GIT_SHALLOW    TRUE
)

FetchContent_MakeAvailable(opencv)

# Manually set some OpenCV projects to be in folders not to pollute the VS solution
set_target_properties(opencv_highgui_plugins PROPERTIES FOLDER "opencv plugins")
set_target_properties(opencv_videoio_plugins PROPERTIES FOLDER "opencv plugins")

# Manual include dirs for OpenCV (could not find better than this hack)
set(OpenCV_INCLUDE_DIRS "")
list(APPEND OpenCV_INCLUDE_DIRS ${OPENCV_CONFIG_FILE_INCLUDE_DIR})
list(APPEND OpenCV_INCLUDE_DIRS ${OPENCV_MODULE_opencv_core_LOCATION}/include)
list(APPEND OpenCV_INCLUDE_DIRS ${OPENCV_MODULE_opencv_imgcodecs_LOCATION}/include)
list(APPEND OpenCV_INCLUDE_DIRS ${OPENCV_MODULE_opencv_imgproc_LOCATION}/include)
list(APPEND OpenCV_INCLUDE_DIRS ${OPENCV_MODULE_opencv_highgui_LOCATION}/include)
list(APPEND OpenCV_INCLUDE_DIRS ${OPENCV_MODULE_opencv_calib3d_LOCATION}/include)
list(APPEND OpenCV_INCLUDE_DIRS ${OPENCV_MODULE_opencv_features2d_LOCATION}/include)
list(APPEND OpenCV_INCLUDE_DIRS ${OPENCV_MODULE_opencv_flann_LOCATION}/include)
list(APPEND OpenCV_INCLUDE_DIRS ${OPENCV_MODULE_opencv_ml_LOCATION}/include)
list(APPEND OpenCV_INCLUDE_DIRS ${OPENCV_MODULE_opencv_video_LOCATION}/include)
list(APPEND OpenCV_INCLUDE_DIRS ${OPENCV_MODULE_opencv_videoio_LOCATION}/include)
