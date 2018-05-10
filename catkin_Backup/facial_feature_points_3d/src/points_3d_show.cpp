
#include"points_3d_show.h"

int mouseX;
int mouseY;
int mouseBtnType;

void onMouse(int event, int x, int y, int flags, void* ustc)
{
    mouseX  = x;
    mouseY  = y;
    mouseBtnType = event;
}

void Points_3d_show::Inital(void)
{
    init_cloud();
    createLookup();
    Init_Dlib();

    outfile.open("/home/zhang/data.txt");
    if(!outfile)
      std::cout<<"error"<<std::endl;

}
void Points_3d_show::init_cloud(void)
{
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud->height = height;
    cloud->width = width;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

//    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
//    viewer->addCoordinateSystem (1.0);

}

void Points_3d_show::createLookup(void)
{
    const float fx = 1.0f / receive_data.cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / receive_data.cameraMatrixColor.at<double>(1, 1);
    const float cx = receive_data.cameraMatrixColor.at<double>(0, 2);
    const float cy = receive_data.cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
     *it = (c - cx) * fx;
    }
}

void Points_3d_show::Init_Dlib(void)
{
    detector = dlib::get_frontal_face_detector();
    dlib::deserialize("shape_predictor_68_face_landmarks.dat") >> predictor;
}

void Points_3d_show::Detect_facial_points(void)
{

     feature_points_vce.clear();  //清除掉原来的特征点坐标
      detected_face=false;

     dlib::cv_image<dlib::bgr_pixel> cimg(color);
     faces = detector(cimg);
       if (faces.size() > 0)
        {
            dlib::full_object_detection shape = predictor(cimg, faces[0]);
            for (unsigned int i = 0; i < 68; ++i)
            {
               cv::circle(color, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), -1);
            }

            feature_points_vce.push_back(cv::Point(shape.part(27).x(), shape.part(27).y())); //27是鼻子最上面
            feature_points_vce.push_back(cv::Point(shape.part(30).x(), shape.part(30).y())); //鼻尖是29和30

            detected_face=true;
       }




}


void Points_3d_show::createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  const float badPoint = std::numeric_limits<float>::quiet_NaN();

  for(int r = 0; r < depth.rows; ++r)
  {
    pcl::PointXYZRGB *itP = &cloud->points[r * depth.cols];
    const uint16_t *itD = depth.ptr<uint16_t>(r);
    const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
    const float y = lookupY.at<float>(0, r);
    const float *itX = lookupX.ptr<float>();

    for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
    {
      register const float depthValue = *itD / 1000.0f;

      if(*itD == 0)
      {
        itP->x = itP->y = itP->z = badPoint;
        itP->rgba = 0;
        continue;
      }
      itP->z = depthValue;
      itP->x = *itX * depthValue;
      itP->y = y * depthValue;
      itP->b = itC->val[0];
      itP->g = itC->val[1];
      itP->r = itC->val[2];
      //itP->a = 255;
    }

   if(detected_face)
   {
    Point1_z=depth.at<uint16_t>((feature_points_vce[0]))/1000.0;

    Point1_x=lookupX.at<float>(0,feature_points_vce[0].x) * Point1_z;

    Point1_y=lookupY.at<float>(0,feature_points_vce[0].y) * Point1_z;


    Point2_z=depth.at<uint16_t>((feature_points_vce[1]))/1000.0;           //鼻尖

    Point2_x=lookupX.at<float>(0,feature_points_vce[1].x) * Point2_z;

    Point2_y=lookupY.at<float>(0,feature_points_vce[1].y) * Point2_z;

    std::cout<<"27: "<<Point1_x<<"  "<<Point1_y<<"  "<<Point1_z<<std::endl;
    std::cout<<"30: "<<Point2_x<<"  "<<Point2_y<<"  "<<Point2_z<<std::endl;

//    viewer->addCoordinateSystem(1.0); //建立空间直角坐标系
//    viewer->setCameraPosition(Point2_x,Point2_y,Point2_z); //设置坐标原点
//    viewer->initCameraParameters();   //初始化相机参数

    outfile<<Point2_x<<" "<<Point2_y<<" "<<Point2_z<<endl;
   }

  }
}

/******************************************************
 *    Function Name: show_clicked_3d_infor
 *    功能:   当单击RGB图片上某个点时,显示出该点的三维坐标
 *           主要是为了看相机测量的精度的,使用这个函数一般是
 *           用相机拍摄标定板,去测量标定板角点之间的距离来
 *           衡量相机的精度
 * ***************************************************/

void Points_3d_show::show_clicked_3d_infor(void)
{
    if(mouseBtnType== cv::EVENT_LBUTTONUP)
    {
        img_x_last=img_x;
        img_y_last=img_y;
        img_x = mouseX;
        img_y = mouseY;
        pt_last=pt_now;
        pt_now = cloud->points[img_y * depth.cols + img_x];
    }
    ossXYZ.str("");
    ossXYZ << "( " << pt_now.x << ", " << pt_now.y<< ", " << pt_now.z << " )";
    cv::putText(color, ossXYZ.str(), cv::Point(img_x+10, img_y), font, 1, colorText, 3, CV_AA);
    cv::circle(color, cv::Point(img_x, img_y), 5, cv::Scalar(0, 255, 0), -1);

    ossXYZ.str("");
    ossXYZ << "( " << pt_last.x << ", " << pt_last.y << ", " << pt_last.z << " )";
    cv::putText(color, ossXYZ.str(), cv::Point(img_x_last+10, img_y_last), font, 1, colorText, 3, CV_AA);
    cv::circle(color, cv::Point(img_x_last, img_y_last), 5, cv::Scalar(0, 255, 0), -1);
    mouseBtnType = cv::EVENT_MOUSEMOVE;
}


void Points_3d_show::cloudViewer()
{

   const std::string window_name = "color viewer";
   cv::namedWindow(window_name,2);
   cv::setMouseCallback(window_name, onMouse, nullptr);    // 注册鼠标回调函数, 第三个参数是C++11中的关键字, 若不支持C++11, 替换成NULL

   for(; ros::ok();)
   {
      if(receive_data.updateCloud)
      {
        receive_data.lock.lock();
        color = receive_data.color;
        depth = receive_data.depth;
        receive_data.updateCloud = false;
        receive_data.lock.unlock();

        //show_clicked_3d_infor();        //使用标定板验证测量是否准确

        Detect_facial_points();

        createCloud(depth, color, cloud);

        viewer.showCloud(cloud);

        //viewer->removePointCloud("sample cloud111");

        // viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud111");

        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "sample cloud111");




        cv::imshow(window_name, color);
        cv::waitKey(1);
      }

   }

   outfile.close();

    cv::destroyAllWindows();
}
