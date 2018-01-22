
#include "hand_eye_calibrator.h"

using namespace std;

#define pi 3.141592653


hand_eye_Calibrator::hand_eye_Calibrator()     //: flag(0), mustInitUndistort(true)
{

    cam_to_base.matrix()<<1.0, 0.0, 0.0, 0.0,  0.0, 1.0, 0.0, -0.2,  0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    end_to_checker.matrix()<<1.0, 0.0, 0.0, 0.0,  0.0, 1.0, 0.0, 0.0,  0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

}


int hand_eye_Calibrator::addChessboardPoints(const std::vector<std::string>& filelist,  cv::Size & boardSize)
{

    std::vector<cv::Point2f> imageCorners;

    std::vector<Eigen::Vector2d> imageCorners_eigen;
    Eigen::Vector2d point_eigen_temp;

    std::vector<Eigen::Vector4d> objectCorners_eigen;

    for (int i=0; i<boardSize.height; i++)
    {
        for (int j=0; j<boardSize.width; j++)
        {
            objectCorners_eigen.push_back(Eigen::Vector4d(j*0.015, i*0.015, 0.0f,1.0));
        }
    }

    cv::Mat image;
    int successes = 0;
    for (int i=0; i<filelist.size(); i++)
    {
        image = cv::imread(filelist[i],0);

        bool found = cv::findChessboardCorners(image, boardSize, imageCorners);

        cv::cornerSubPix(image, imageCorners, cv::Size(5,5),  cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,30, 0.1));  		// max number of iterations

        if (imageCorners.size() == boardSize.area())
        {
            //把cv::point转换成 eigen的vector2d
            for(int i =0;i<imageCorners.size();i++)
            {
              point_eigen_temp<< imageCorners[i].x,imageCorners[i].y ;
              imageCorners_eigen.push_back(point_eigen_temp);
            }

            imagePoints.push_back(imageCorners);

            imagePoints_eigen.push_back(imageCorners_eigen);
            objectPoints_eigen.push_back(objectCorners_eigen);

            successes++;
         }
        cv::drawChessboardCorners(image, boardSize, imageCorners, found);
        cv::imshow("Corners on Chessboard", image);
        cv::waitKey(20);
    }   
    return successes;
}



int hand_eye_Calibrator::readTransformPairsFromFile(std::string filename)
{

    cv::FileStorage fs(filename, cv::FileStorage::READ);

    int frameCount;

    fs["frameCount"] >> frameCount;

    if(fs.isOpened())
    {
        for (int i = 0; i < frameCount; ++i)

            {
                cv::Mat_<double> t1cv = cv::Mat_<double>::ones(4,4);
                std::stringstream ss1;
                ss1 << "T1_" << i;
                fs[ss1.str()] >> t1cv;
                Eigen::Affine3d t1e;
                cv::cv2eigen(t1cv,t1e.matrix());
                armEnd_Pose_eigen.push_back(t1e);
             }
         fs.release();
    }
    else
    {
        std::cerr << "failed to open input file " << filename << "\n";
        return 1;
    }
    return 0;
}



void hand_eye_Calibrator:: get_size_of_data()
   {
       cout<<"size of objectPoints_eigen:"<<objectPoints_eigen.size()<<endl;
       //cout<<"size of imagePoints:       "<<imagePoints.size()<<endl;
       cout<<"size of armEnd_Pose_eigen: "<<armEnd_Pose_eigen.size()<<endl;
   }


void hand_eye_Calibrator:: calculate(void)
{ 
   ceres::Problem problem;

   double camera[12]={0,0,1,0,-0.2,1.0,0,0,1.0,0,0,0};
   double object_point[3];
   double base_to_end[16];
   char count=0;

   for(int i=0;i<objectPoints_eigen.size();i++)
     for(int j=0;j<objectPoints_eigen[i].size();j++)
     {
         object_point[0]=objectPoints_eigen[i][j](0,0);
         object_point[1]=objectPoints_eigen[i][j](1,0);
         object_point[2]=objectPoints_eigen[i][j](2,0);
         for(int ii=0;ii<4;ii++)
             for(int jj=0;jj<4;jj++)
         {
             base_to_end[count]=armEnd_Pose_eigen[i](ii,jj);
             ++count;
         }

         ceres::CostFunction* cost_function =ReprojectionError::Create(imagePoints[i][j].x,imagePoints[i][j].y);
         problem.AddResidualBlock(cost_function,
                                      NULL /* squared loss */,
                                      camera,
                                     object_point,
                                     base_to_end);

     }

    ceres::Solver::Options options;
     options.linear_solver_type = ceres::DENSE_SCHUR;
     options.minimizer_progress_to_stdout = true;

     ceres::Solver::Summary summary;
     ceres::Solve(options, &problem, &summary);
     std::cout << summary.FullReport() << "\n";

   /*Eigen::Vector4d project_temp_4x1;
   for(int i=0;i<objectPoints_eigen.size();i++)
   for(int j=0;j<objectPoints_eigen[i].size();j++)
   {
       project_temp_4x1= cam_to_base*armEnd_Pose_eigen[i]*end_to_checker*objectPoints_eigen[i][j];

   }*/
}










