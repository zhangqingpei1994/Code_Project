
#include"points_3d_show.h"



//pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

int mouseX;
int mouseY;
int mouseBtnType;

void onMouse_click(int event, int x, int y, int flags, void* ustc)
{
    mouseX  = x;
    mouseY  = y;
    mouseBtnType = event;
}


void Points_3d_show::init_Point_3d_show(size_t width_in,size_t height_in,cv::Mat cameraRGBinfor_in,cv::Mat cameraDepthinfor_in)
{
      width=width_in;
      height=height_in;
      cameraRGBinfor=cameraRGBinfor_in;
      cameraDepthinfor=cameraDepthinfor_in;
      createLookup();
      init_cloud();
      Init_Dlib();
}

void Points_3d_show::init_cloud(void)
{
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud->height = height;
    cloud->width = width;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
}


void Points_3d_show::createLookup(void)
{
    const float fx = 1.0f / cameraDepthinfor.at<double>(0, 0);
    const float fy = 1.0f / cameraDepthinfor.at<double>(1, 1);
    const float cx = cameraDepthinfor.at<double>(0, 2);
    const float cy =cameraDepthinfor.at<double>(1, 2);
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


void Points_3d_show::Detect_facial_points(cv::Mat & color)
{
    dlib::cv_image<dlib::bgr_pixel> cimg(color);
    faces = detector(cimg);
    have_detect_face=false;
    if (faces.size() > 0)
    {
      have_detect_face=true;
      nose_position.clear();

      dlib::full_object_detection shape = predictor(cimg, faces[0]);
      for (unsigned int i = 0; i < 68; ++i)        //鼻尖是29和30
      {
         cv::circle(color, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), -1);
      }

      nose_position.push_back(cv::Point(shape.part(27).x(), shape.part(27).y()));
      nose_position.push_back(cv::Point(shape.part(30).x(), shape.part(30).y()));

    }
}

void Points_3d_show::createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) const
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
    }
  }
}

void Points_3d_show::get_Nose_Position(Eigen::Vector4d & nose1,Eigen::Vector4d & nose2)
{
    if(have_detect_face)
    {
        pcl::PointXYZRGB nose_pointtemp1;
        pcl::PointXYZRGB nose_pointtemp2;
        nose_pointtemp1=cloud->points[nose_position[0].y * width + nose_position[0].x];
        nose_pointtemp2=cloud->points[nose_position[1].y * width + nose_position[1].x];

        nose1(0,0)=nose_pointtemp1.x;
        nose1(1,0)=nose_pointtemp1.y;
        nose1(2,0)=nose_pointtemp1.z;
        nose1(3,0)=1.0;

        nose2(0,0)=nose_pointtemp2.x;
        nose2(1,0)=nose_pointtemp2.y;
        nose2(2,0)=nose_pointtemp2.z;
        nose2(3,0)=1.0;

        //std::cout<<nose_pointtemp1.x<<"  "<<nose_pointtemp1.y<<"  "<<nose_pointtemp1.z<<std::endl;
        //std::cout<<nose_pointtemp2.x<<"  "<<nose_pointtemp2.y<<"  "<<nose_pointtemp2.z<<std::endl;
     }
}

void Points_3d_show::cloudViewer(cv::Mat& color_in,cv::Mat& depth_in)
{
        Detect_facial_points(color_in);

        createCloud(depth_in, color_in, cloud);

        /*const std::string window_name = "color viewer";
        cv::namedWindow(window_name,2);
        cv::setMouseCallback(window_name, onMouse_click, nullptr);    // 注册鼠标回调函数, 第三个参数是C++11中的关键字, 若不支持C++11, 替换成NULL
        show_clicked_3d_infor(color_in,depth_in);
        cv::imshow(window_name, color_in);
        cv::waitKey(1);*/

        //viewer.showCloud(cloud);
}
/******************************************************
 *    Function Name: show_clicked_3d_infor
 *    功能:   当单击RGB图片上某个点时,显示出该点的三维坐标
 *           主要是为了看相机测量的精度的,使用这个函数一般是
 *           用相机拍摄标定板,去测量标定板角点之间的距离来
 *           衡量相机的精度
 * ***************************************************/
void Points_3d_show::show_clicked_3d_infor(cv::Mat& color,cv::Mat depth)
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
    cv::putText(color, ossXYZ.str(), cv::Point(img_x+10, img_y), font, 0.5, colorText, 1, CV_AA);
    cv::circle(color, cv::Point(img_x, img_y), 5, cv::Scalar(0, 255, 0), -1);

    ossXYZ.str("");
    ossXYZ << "( " << pt_last.x << ", " << pt_last.y << ", " << pt_last.z << " )";
    cv::putText(color, ossXYZ.str(), cv::Point(img_x_last+10, img_y_last), font, 0.5, colorText, 1, CV_AA);
    cv::circle(color, cv::Point(img_x_last, img_y_last), 5, cv::Scalar(0, 255, 0), -1);
    mouseBtnType = cv::EVENT_MOUSEMOVE;
}
