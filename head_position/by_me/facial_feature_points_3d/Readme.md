自己写的Ros包,订阅iai_kinect2的话题,并把程序分块了,写成了好几个cpp的形式

各个cpp介绍:

grab_image.cpp: 订阅话题,得到Kinect2的RGB和Depth图像,并获得相机的信息

points_3d_show.cpp:   使用Dlib库对人脸特征点进行定位,并获得脸部的三维点云以及脸部特征点对应的点云.

main.cpp  主程序的执行

在这个包中,为了加速程序执行时间,开辟了多线程
