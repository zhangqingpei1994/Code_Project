include_directories(${CMAKE_SOURCE_DIR}/kfusion/include)

file(GLOB srcs *.cpp *.hpp)     #把当前文件夹下的源码列表（文件后缀匹配的那些文件）存到变量 srcs 中
add_executable(demo ${srcs})
target_link_libraries(demo ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CUDA_LIBRARIES} ${freenect2_LIBRARIES} kfusion)


#set_target_properties(demo PROPERTIES
#    DEBUG_POSTFIX "d"
#    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
#    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

#install(TARGETS demo RUNTIME DESTINATION bin COMPONENT main)
#install(FILES ${srcs} DESTINATION app COMPONENT main)
