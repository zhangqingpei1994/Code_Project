
#include"track_head.h"

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


void Track_head::init_track_head(size_t width_in,size_t height_in,cv::Mat cameraRGBinfor_in,cv::Mat cameraDepthinfor_in)
{
      width=width_in;
      height=height_in;
      cameraRGBinfor=cameraRGBinfor_in;
      cameraDepthinfor=cameraDepthinfor_in;
      createLookup();
      init_cloud();
      Init_Dlib();
}

void Track_head::init_cloud(void)
{
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud->height = height;
    cloud->width = width;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
}

void Track_head::createLookup(void)
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

void Track_head::Init_Dlib(void)
{
    detector = dlib::get_frontal_face_detector();
    dlib::deserialize("shape_predictor_68_face_landmarks.dat") >> predictor;
}

/*********************************************************************
 * @brief Track_head::Detect_facial_points
 * @param color:  inpute param
 * introduction:  在color的特征点部位画出圆点,并将要存的目标点保存到了
 *                nose_position中,所有操作都会改变color
 *
 *********************************************************************/
void Track_head::Detect_facial_points(cv::Mat & color)
{
    dlib::cv_image<dlib::bgr_pixel> cimg(color);
    faces = detector(cimg);
    have_detect_face=false;
    if (faces.size() > 0)
    {
      have_detect_face=true;
      coordinate_point_2d.clear();

      dlib::full_object_detection shape = predictor(cimg, faces[0]);
      for (unsigned int i = 0; i < 68; ++i)        //鼻尖是29和30
      {
         cv::circle(color, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), -1);
      }

      coordinate_point_2d.push_back(cv::Point(shape.part(27).x(), shape.part(27).y()));
      coordinate_point_2d.push_back(cv::Point(shape.part(28).x(), shape.part(28).y()));
      coordinate_point_2d.push_back(cv::Point(shape.part(29).x(), shape.part(29).y()));
      coordinate_point_2d.push_back(cv::Point(shape.part(30).x(), shape.part(30).y()));
      coordinate_point_2d.push_back(cv::Point(shape.part(36).x(), shape.part(36).y()));
      coordinate_point_2d.push_back(cv::Point(shape.part(39).x(), shape.part(39).y()));
      coordinate_point_2d.push_back(cv::Point(shape.part(42).x(), shape.part(42).y()));
      coordinate_point_2d.push_back(cv::Point(shape.part(45).x(), shape.part(45).y()));

      std::cout<<"coordinate_point_2d.size: "<<coordinate_point_2d.size()<<std::endl;

    }
}

/*************************************************************************
 * @brief Track_head::createCloud
 * @param depth:    no change
 * @param color:    no change
 * @param cloud:    changed
 *************************************************************************/
void Track_head::createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) const
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


bool Track_head::detect_face(cv::Mat& color_in,cv::Mat& depth_in)
{
        Detect_facial_points(color_in);

        createCloud(depth_in, color_in, cloud);

        /*const std::string window_name = "color viewer";
        cv::namedWindow(window_name,2);
        cv::setMouseCallback(window_name, onMouse_click, nullptr);    // 注册鼠标回调函数, 第三个参数是C++11中的关键字, 若不支持C++11, 替换成NULL
        show_clicked_3d_infor(color_in,depth_in);
        cv::imshow(window_name, color_in);
        cv::waitKey(1);
        viewer.showCloud(cloud);*/

        return have_detect_face;
}


void Track_head::get_Head_Coordinate(void)
{
    if(have_detect_face)
    {
        pcl::PointXYZRGB point_temp;

        Eigen::Vector3d point_temp_v3d;

        coordinate_point_3d.clear();

        for(int i=0;i<coordinate_point_2d.size();i++)
        {
            point_temp=cloud->points[coordinate_point_2d[i].y * width + coordinate_point_2d[i].x];
            point_temp_v3d(0,0)=point_temp.x;
            point_temp_v3d(1,0)=point_temp.y;
            point_temp_v3d(2,0)=point_temp.z;
            coordinate_point_3d.push_back(point_temp_v3d);
        }

         Eigen::Vector3d point_vec_tempX1,point_vec_tempY1,point_vec_tempX2,point_vec_tempY2;

         point_vec_tempX1=coordinate_point_3d[3]-coordinate_point_3d[0];
         point_vec_tempX2=coordinate_point_3d[2]-coordinate_point_3d[1];
         head_X_inCam=(point_vec_tempX1+point_vec_tempX1)/2.0;

         point_vec_tempY1=coordinate_point_3d[7]-coordinate_point_3d[4];
         point_vec_tempY2=coordinate_point_3d[6]-coordinate_point_3d[5];
         head_Y_inCam=(point_vec_tempY1+point_vec_tempY2)/2.0;

         head_Z_inCam=head_X_inCam.cross(head_Y_inCam);

         head_X_inCam/=head_X_inCam.norm();

         head_Y_inCam/=head_Y_inCam.norm();

         head_Z_inCam/=head_Z_inCam.norm();

         head_ori=(coordinate_point_3d[7]+coordinate_point_3d[6]+coordinate_point_3d[5]+coordinate_point_3d[4])/4.0;

         T_head_to_cam=Eigen::Isometry3d::Identity();
         for(int i=0;i<3;i++)
         {
           T_head_to_cam(i,0)=head_X_inCam(i,0);
           T_head_to_cam(i,1)=head_Y_inCam(i,0);
           T_head_to_cam(i,2)=head_Z_inCam(i,0);
           T_head_to_cam(i,3)=head_ori(i,0);
         }

         std::cout<<"head to cam: "<<std::endl<<T_head_to_cam.matrix()<<std::endl;

     }
}

void Track_head::track_head( double *target)
{
      get_Head_Coordinate();

      T_illpoint_to_base= cam_to_armbase * T_head_to_cam* T_illpoint_to_head ;

      std::cout<<"T_illpoint_to_base:"<<std::endl<<T_illpoint_to_base.matrix()<<std::endl;

      cv::Mat R_matrix = (cv::Mat_<double>(3,3) << T_illpoint_to_base(0,0),  T_illpoint_to_base(0,1),  T_illpoint_to_base(0,2),
                                                   T_illpoint_to_base(1,0),  T_illpoint_to_base(1,1),  T_illpoint_to_base(1,2),
                                                   T_illpoint_to_base(2,0),  T_illpoint_to_base(2,1),  T_illpoint_to_base(2,2));
      cv::Mat R_vec;
      cv::Rodrigues(R_matrix, R_vec);

      target[0]=T_illpoint_to_base(0,3);
      target[1]=T_illpoint_to_base(1,3);
      target[2]=T_illpoint_to_base(2,3);
      target[3]=R_vec.at<double>(0,0);
      target[4]=R_vec.at<double>(1,0);
      target[5]=R_vec.at<double>(2,0);
}


void Track_head::get_end_to_base(double ur5_Px,double ur5_Py,double ur5_Pz,double ur5_Rx,double ur5_Ry,double ur5_Rz)
{
    cv::Mat R_vector = (cv::Mat_<double>(3,1) << ur5_Rx, ur5_Ry, ur5_Rz );
    cv::Mat R;
    cv::Rodrigues(R_vector, R);
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);
    T_end_to_base=Eigen::Isometry3d::Identity();
    T_end_to_base.prerotate(r);
    T_end_to_base.pretranslate(Eigen::Vector3d(ur5_Px,ur5_Py,ur5_Pz));

}

void Track_head::get_illpoint_to_head(void)
{
      get_Head_Coordinate();

      T_illpoint_to_head= T_head_to_cam.inverse() * armbase_to_cam *T_end_to_base;

      std::cout<<"illpoint_to_head: "<<std::endl<<T_illpoint_to_head.matrix()<<std::endl;

}

/******************************************************
 *    Function Name: show_clicked_3d_infor
 *    功能:   当单击RGB图片上某个点时,显示出该点的三维坐标
 *           主要是为了看相机测量的精度的,使用这个函数一般是
 *           用相机拍摄标定板,去测量标定板角点之间的距离来
 *           衡量相机的精度
 * ***************************************************/
void Track_head::show_clicked_3d_infor(cv::Mat& color,cv::Mat depth)
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
