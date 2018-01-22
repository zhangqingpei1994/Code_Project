
#include <iomanip>
#include "hand_eye_calibrator.h"

#include <chrono>

using namespace cv;
using namespace std;

struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST ( double x, double y ) : _x ( x ), _y ( y ) {}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const abc,     // 模型参数，有3维
        T* residual ) const     // 残差
    {
        residual[0] = T ( _y ) - ceres::exp ( abc[0]*T ( _x ) *T ( _x ) + abc[1]*T ( _x ) + abc[2] ); // y-exp(ax^2+bx+c)
        return true;
    }
    const double _x, _y;    // x,y数据
};


int main()
{

       Mat image=imread("image01.jpg",0);
       imshow("original",image);

       int picture_number;

       hand_eye_Calibrator cameraCali;
       Size boardSize(11,8);
       Size imageSize=image.size();


       const string str[]={"image01.jpg","image02.jpg","image03.jpg","image04.jpg","image05.jpg","image06.jpg","image07.jpg","image08.jpg","image09.jpg","image10.jpg",
                           "image11.jpg","image12.jpg","image13.jpg","image14.jpg","image15.jpg","image16.jpg","image17.jpg","image18.jpg","image19.jpg","image20.jpg",
                           "image21.jpg","image22.jpg","image23.jpg","image24.jpg","image25.jpg","image26.jpg","image27.jpg","image28.jpg","image29.jpg","image30.jpg",
                           "image31.jpg","image32.jpg","image33.jpg","image34.jpg","image35.jpg","image36.jpg","image37.jpg","image38.jpg","image39.jpg","image40.jpg"};

       std::vector<std::string> filename(str,str+25);          //+n 就是用n张照片

       picture_number=cameraCali.addChessboardPoints(filename,boardSize);

       cameraCali.readTransformPairsFromFile("TransformPairsInput.yml");

       cameraCali.get_size_of_data();

       cameraCali.calculate();

       //ceres::Problem problem;

       //cameraCali.calibrate(imageSize);
       //image=cameraCali.remap(image);
       //imshow("dealed image",image);

       waitKey();


    return 0;
}



