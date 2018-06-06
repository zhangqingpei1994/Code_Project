KinFu --by zhang
============

 

Key changes/features:
* Performance has been improved by 1.6x factor (Fermi-tested)
* Code size is reduced drastically. Readability improved. 
* No hardcoded algorithm parameters! All of them can be changed at runtime (volume size, etc.)
* The code is made independent from OpenCV GPU module and PCL library. 

Dependencies:
* Fermi or Kepler or newer
* CUDA 5.0 or higher
* OpenCV 3.2.4 with new Viz module (only opencv_core, opencv_highgui, opencv_imgproc, opencv_viz modules required). 
* libfreenect2

Implicit dependency (needed by opencv_viz):
* VTK 5.8.0 or higher. (apt-get install on linux, for windows please download and compile from www.vtk.org)

### note:
这个工程是 使用opencv3.2.4写的,之后的代码自己改成了opencv2.4.13,适应小林的程序的版本要求.里面的相机驱动程序换成了Kinect2的,关于kinect1的东西全部删掉了,能实现最开始的功能,而且做到了最简,并可以生成opencv3.2.4版本的kfusion的动态库,在外部程序使用这个动态库的时候,opencv版本一定要使用3的才能成功


