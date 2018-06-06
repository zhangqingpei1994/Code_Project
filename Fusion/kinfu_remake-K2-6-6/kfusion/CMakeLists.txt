set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")

add_module_library(kfusion)
target_link_libraries(kfusion ${CUDA_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${OpenCV_LIBS}  ${freenect2_LIBRARIES})

file(GLOB srcs1 src/cuda/*.cu  )
cuda_add_library(kfusion12 SHARED ${srcs1})
target_link_libraries(kfusion12 ${CUDA_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${OpenCV_LIBS}  ${freenect2_LIBRARIES})

file(GLOB srcs src/*.cpp )
add_library(kfusion11 SHARED ${srcs})
target_link_libraries(kfusion11 ${CUDA_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${OpenCV_LIBS}  ${freenect2_LIBRARIES} kfusion12)




