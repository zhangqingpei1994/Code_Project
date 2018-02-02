/*************************************************************************
	> File Name: rgbd-slam-tutorial-gx/part III/code/include/slamBase.h
	> Author: xiang gao
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年07月18日 星期六 15时14分22秒
    > 说明：rgbd-slam教程所用到的基本函数（C风格）
 ************************************************************************/
# pragma once

// 各种头文件 
// C++标准库
#include <fstream>
#include <vector>
#include <map>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/timer.hpp>

// 类型定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

extern PointCloud::Ptr output ;

// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS 
{ 
    double cx, cy, fx, fy, scale;
};

// 帧结构
struct FRAME
{
    cv::Mat rgb, depth; //该帧对应的彩色图与深度图
    cv::Mat desp;       //特征描述子
    vector<cv::KeyPoint> kp; //关键点
};

// PnP 结果
struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
};


// 参数读取类
class ParameterReader
{
public:
    ParameterReader( string filename="/home/zhang/pclKinFusion/multiImage/parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};




class joinPointCloud
{
public:
    joinPointCloud ();
        // image2PonitCloud 将rgb图转换为点云
    PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera );
    // point2dTo3d 将单个点从图像坐标转换为空间坐标
    // input: 3维点Point3f (u,v,d)
    cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera );
    // computeKeyPointsAndDesp 同时提取关键点与特征描述子
    void computeKeyPointsAndDesp(FRAME& frame);
    // estimateMotion 计算两个帧之间的运动
    // 输入：帧1和帧2, 相机内参
    RESULT_OF_PNP estimateMotion(FRAME& frame1);

    void joinPointCloud_run(FRAME &frame_now);

    void setFram_last(FRAME fram_now);



private:
    CAMERA_INTRINSIC_PARAMETERS camera; //相机参数
    ParameterReader pd;                 //参数读取类
    string detecter;
    string descriptor;
    Eigen::Isometry3d T_final = Eigen::Isometry3d::Identity();
    double good_match_threshold ;
    double camera_matrix_data[3][3] ;
    PointCloud::Ptr cloud_last;
    FRAME  frame_last;


};
