
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


void Points_3d_show::init_cloud(void)
{
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud->height = height;
    cloud->width = width;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
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
    dlib::cv_image<dlib::bgr_pixel> cimg(color);
     faces = detector(cimg);
       if (faces.size() > 0)
        {
            dlib::full_object_detection shape = predictor(cimg, faces[0]);
            for (unsigned int i = 0; i < 68; ++i)       //鼻尖是29和30
            {
               cv::circle(color, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), -1);
            }
       }
}



void Points_3d_show::createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
{
  const float badPoint = std::numeric_limits<float>::quiet_NaN();

  for(int r = 0; r < depth.rows; ++r)
  {
    pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
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
      itP->a = 255;
    }
  }
}


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

        //show_clicked_3d_infor();    //使用标定板验证测量是否准确

        Detect_facial_points();

        createCloud(depth, color, cloud);


        viewer.showCloud(cloud);

        cv::imshow(window_name, color);
        cv::waitKey(1);
      }

   }
    cv::destroyAllWindows();
}
