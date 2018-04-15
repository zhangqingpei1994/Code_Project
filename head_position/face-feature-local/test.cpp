#include <dlib/opencv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>

using namespace dlib;
using namespace std;



int main()
{
    cout<<"aaa";
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        cerr << "Unable to connect to camera" << endl;
        return EXIT_FAILURE;
    }
    //需要一个人脸检测器，获得一个边界框
    frontal_face_detector detector = get_frontal_face_detector();


     //预测特征点位置用的
    shape_predictor predictor;
    deserialize("shape_predictor_68_face_landmarks.dat") >> predictor;

    image_window win;          //显示检测到的人脸
    image_window win11;
    //image_window winhog(draw_fhog(hog));

    while (1)
    {
        //使用相机的时候用的
        cv::Mat temp;
        cap >> temp;
        cv_image<bgr_pixel> cimg(temp);
        std::vector<rectangle> faces = detector(cimg);   //检测人脸 */

        //使用本地图片的时候用的
        /*cv::Mat temp_local=cv::imread("indoor_002.png",1);

        cv_image<bgr_pixel> cimg(temp_local);
        std::vector<rectangle> faces = detector(cimg);   //检测人脸*/

       /* array2d<rgb_pixel> img;
        dlib::assign_image(img, cimg);
        //单个hog特征是31维的…
        array2d<matrix<float, 31, 1> > hog;
        extract_fhog_features(img, hog,16);//默认为8
        //image_window winhog(draw_fhog(hog));
        win11.clear_overlay();
        win11.set_image(draw_fhog(hog));*/


        // Find the feature points of each face
        if (faces.size() > 0)
        {

            full_object_detection shape = predictor(cimg, faces[0]);

            for (unsigned int i = 0; i < 68; ++i)
            {
                cv::circle(temp, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), -1);
            }

        }

        win.clear_overlay();
        win.set_image(cimg);
        win.add_overlay(faces, rgb_pixel(255,0,0));

        cv::imshow("demo", temp);
        //cv::waitKey(0);



        if (cv::waitKey(5) == 27)
        {
            break;
        }

    }

    return 0;
}

