/*************************************************************************
	> File Name: src/slamBase.cpp
	> Author: xiang gao
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
    > Implementation of slamBase.h
	> Created Time: 2015年07月18日 星期六 15时31分49秒
 ************************************************************************/

#include "slamBase.h"
#include <opencv2/core/eigen.hpp>


using namespace cv;

PointCloud::Ptr output (new PointCloud());

 joinPointCloud::joinPointCloud(void)
{
    detecter = pd.getData( "detector" );
    descriptor = pd.getData( "descriptor" );
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );
    good_match_threshold = atof( pd.getData( "good_match_threshold" ).c_str() );
    camera_matrix_data[0][0] =camera.fx;
    camera_matrix_data[0][2] =camera.cx;
    camera_matrix_data[1][1] =camera.fy;
    camera_matrix_data[1][2] =camera.cy;
    camera_matrix_data[2][2] =1;

}

  void joinPointCloud::setFram_last(FRAME fram_now)
  {
      frame_last=fram_now;
      cout<<frame_last.kp.size()<<endl;
  }

 //这个函数将近20ms 用960的图像的时候
PointCloud::Ptr joinPointCloud::image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    PointCloud::Ptr cloud ( new PointCloud );

    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

cv::Point3f joinPointCloud::point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    cv::Point3f p; // 3D 点
    p.z = double( point.z ) / camera.scale;
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;
    return p;
}

// computeKeyPointsAndDesp 同时提取关键点与特征描述子
void joinPointCloud::computeKeyPointsAndDesp( FRAME& frame )
{

    
    Ptr<ORB> _detector = ORB::create();
    _detector->detect( frame.rgb, frame.kp );
    _detector->compute( frame.rgb, frame.kp, frame.desp );

    //return;
}

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2
// 输出：rvec 和 tvec
RESULT_OF_PNP joinPointCloud::estimateMotion( FRAME& frame1 )
{
    vector< cv::DMatch > matches;
    cv::BFMatcher matcher;
    //imshow("test1",frame1.rgb);
    //waitKey(1);
    //imshow("test2",frame_last.rgb);
    matcher.match( frame1.desp, frame_last.desp, matches );
   
    cout<<"find total "<<matches.size()<<" matches."<<endl;
    vector< cv::DMatch > goodMatches;
    double minDis = 9999;


    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }
    if(minDis==0)
        minDis=0.1;
    cout<<minDis<<endl;

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < good_match_threshold*minDis)
            goodMatches.push_back( matches[i] );
    }
    cout<<goodMatches.size()<<endl;
    cv::Mat imgMatches;
    cv::drawMatches( frame1.rgb, frame1.kp, frame_last.rgb, frame_last.kp, goodMatches, imgMatches );
    //cv::imshow( "good_matches", imgMatches );
    //waitKey();

    cout<<"good matches: "<<goodMatches.size()<<endl;
    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;

    // 相机内参
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( frame_last.kp[goodMatches[i].trainIdx].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, camera );   //pd是相机坐标系下的三維坐標
        pts_obj.push_back( pd );
    }


    cout<<"solving pnp"<<endl;
    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;   //rvec是3X1的旋转向量，tvec是3X1的平移向量
    // 求解pnp，相机坐标系下的三维坐标和相机坐标系下的像素坐标之间的转换关系
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers );

    RESULT_OF_PNP result;
    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;
    return result;
}

void joinPointCloud::joinPointCloud_run(FRAME & frame_now)
{

  cout<<"extracting features"<<endl;
  computeKeyPointsAndDesp( frame_now);

  cout<<"solving pnp"<<endl;
  if(frame_last.kp.size()==0)
  {
      //frame_last=frame_now;
      frame_now.rgb.copyTo(frame_last.rgb);
      frame_now.depth.copyTo(frame_last.depth);
      frame_last.kp=frame_now.kp;
      frame_now.desp.copyTo(frame_last.desp);
      PointCloud::Ptr first_Fram_cloud = image2PointCloud( frame_now.rgb, frame_now.depth, camera );
       *output += *first_Fram_cloud;

      return;
  }
   RESULT_OF_PNP result = estimateMotion( frame_now);
   cout<<result.rvec<<endl<<result.tvec<<endl;


   // 处理result
  // 将旋转向量转化为旋转矩阵
   cv::Mat R;
  cv::Rodrigues( result.rvec, R );
  Eigen::Matrix3d r;  //3*3 double型矩阵
  cv::cv2eigen(R, r);
  // 将平移向量和旋转矩阵转换成变换矩阵
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();   //4X4等距矩阵
  Eigen::AngleAxisd angle(r);     //3*1旋转向量
  cout<<"translation"<<endl;
  T = angle;
  T(0,3) = result.tvec.at<double>(0,0);
  T(1,3) = result.tvec.at<double>(0,1);
  T(2,3) = result.tvec.at<double>(0,2);


  // 转换点云
  cout<<"converting image to clouds"<<endl;
  PointCloud::Ptr cloud_now_temp (new PointCloud());


  PointCloud::Ptr cloud_now = image2PointCloud( frame_now.rgb, frame_now.depth, camera );
  T_final.matrix()= T.matrix()*T_final.matrix();                                  //这个乘的顺序有待确定
  pcl::transformPointCloud( *cloud_now, *cloud_now_temp, T_final.matrix() );
  *output += *cloud_now_temp;


  /*PointCloud::Ptr cloud_now = image2PointCloud( frame_now.rgb, frame_now.depth, camera );
  pcl::transformPointCloud( *cloud_now, *cloud_now_temp, T.matrix() );
  *output += *cloud_now_temp;*/



  frame_now.rgb.copyTo(frame_last.rgb);
  frame_now.depth.copyTo(frame_last.depth);
  frame_last.kp=frame_now.kp;
  frame_now.desp.copyTo(frame_last.desp);


}
