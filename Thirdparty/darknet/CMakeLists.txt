cmake_minimum_required(VERSION 2.8.11)

#Suppressing CMAKE 3.0 warnings
if(POLICY CMP0043)
    cmake_policy(SET CMP0037 OLD)
endif()

add_definitions(-std=c++11)

set(PROJECT_NAME darknet)

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/)



project(${PROJECT_NAME})

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#OpenCV package
set(OpenCV_DIR /home/wchen/opencv-3.1.0/build/)
find_package(OpenCV 3.1.0 REQUIRED)
MESSAGE("OpenCV version: ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})
add_definitions(-DOPENCV)

#CUDA package
set(CUDA_DIR /usr/local/cuda/)
find_package(CUDA REQUIRED)
include_directories(${CUDA_DIR}/include)
add_definitions(-DGPU)

#cudnn package
find_package(cuDNN REQUIRED)
if(CUDNN_FOUND)
    include_directories(${CUDNN_INCLUDE_DIRS})
    add_definitions(-DCUDNN)
endif()


find_package(Threads REQUIRED)

include_directories(./src)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(YOLO_CUDA_FILES 
src/activation_kernels.cu
src/avgpool_layer_kernels.cu
src/blas_kernels.cu
src/col2im_kernels.cu
src/convolutional_kernels.cu
src/crop_layer_kernels.cu
src/dropout_layer_kernels.cu
src/deconvolutional_kernels.cu
src/im2col_kernels.cu
src/maxpool_layer_kernels.cu
src/network_kernels.cu)

set(YOLO_SOURCE_FILES 
src/activation_layer.cpp
src/activation_layer.h
src/activations.cpp
src/activations.h
src/art.cpp
src/avgpool_layer.cpp
src/avgpool_layer.h
src/batchnorm_layer.cpp
src/batchnorm_layer.h
src/blas.cpp
src/blas.h
src/box.cpp
src/box.h
src/captcha.cpp
src/cifar.cpp
src/classifier.cpp
src/classifier.h
src/coco.cpp
src/col2im.cpp
src/col2im.h
src/compare.cpp
src/connected_layer.cpp
src/connected_layer.h
src/convolutional_layer.cpp
src/convolutional_layer.h
src/cost_layer.cpp
src/cost_layer.h
src/crnn_layer.cpp
src/crnn_layer.h
src/crop_layer.cpp
src/crop_layer.h
src/cuda.cpp
src/cuda.h
src/darknet.cpp
src/data.cpp
src/data.h
src/deconvolutional_layer.cpp
src/deconvolutional_layer.h
src/demo.cpp
src/demo.h
src/detection_layer.cpp
src/detection_layer.h
src/detector.cpp
src/dice.cpp
src/dropout_layer.cpp
src/dropout_layer.h
src/gemm.cpp
src/gemm.h
src/go.cpp
src/gru_layer.cpp
src/gru_layer.h
src/im2col.cpp
src/im2col.h
src/image.cpp
src/image.h
src/layer.cpp
src/layer.h
src/list.cpp
src/list.h
src/local_layer.cpp
src/local_layer.h
src/matrix.cpp
src/matrix.h
src/maxpool_layer.cpp
src/maxpool_layer.h
src/network.cpp
src/network.h
src/nightmare.cpp
src/normalization_layer.cpp
src/normalization_layer.h
src/object.h
src/option_list.cpp
src/option_list.h
src/parser.cpp
src/parser.h
src/region_layer.cpp
src/region_layer.h
src/reorg_layer.cpp
src/reorg_layer.h
src/rnn.cpp
src/rnn_layer.cpp
src/rnn_layer.h
src/rnn_vid.cpp
src/route_layer.cpp
src/route_layer.h
src/shortcut_layer.cpp
src/shortcut_layer.h
src/softmax_layer.cpp
src/softmax_layer.h
src/stb_image.h
src/stb_image_write.h
src/super.cpp
src/swag.cpp
src/tag.cpp
src/tree.cpp
src/tree.h
src/utils.cpp
src/utils.h
src/voxel.cpp
src/writing.cpp
src/yolo.cpp
src/yolo.h)

CUDA_ADD_LIBRARY(darknet SHARED ${YOLO_CUDA_FILES} ${YOLO_SOURCE_FILES})
target_link_libraries(
        ${PROJECT_NAME}
        ${OpenCV_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${CUDA_CUBLAS_LIBRARIES}
        ${CUDA_curand_LIBRARY}
        ${CMAKE_THREAD_LIBS_INIT}
        ${CUDNN_LIBRARIES})
