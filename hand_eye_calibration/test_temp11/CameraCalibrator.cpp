

#include "CameraCalibrator.h"

using namespace std;
using namespace cv;


int CameraCalibrator::addChessboardPoints(const std::vector<std::string>& filelist,  cv::Size & boardSize)
{

	// the points on the chessboard
    std::vector<cv::Point2f> imageCorners;
    std::vector<cv::Point3f> objectCorners;


	// The corners are at 3D location (X,Y,Z)= (i,j,0)
    for (int i=0; i<boardSize.height; i++)
    {
		for (int j=0; j<boardSize.width; j++) 
		{

            objectCorners.push_back(cv::Point3f(i*0.015, j*0.015, 0.0f));
		}
    }

    // 2D Image points:
    cv::Mat image;                   // to contain chessboard image
    int successes = 0;
    // for all viewpoints
    for (int i=0; i<filelist.size(); i++) 
	{

        // Open the image
        image = cv::imread(filelist[i],0);

        // Get the chessboard corners
        bool found = cv::findChessboardCorners(image, boardSize, imageCorners);

        // Get subpixel accuracy on the corners
        cv::cornerSubPix(image, imageCorners, cv::Size(5,5),  cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,30, 0.1));  		// max number of iterations


        // If we have a good board, add it to our data
        if (imageCorners.size() == boardSize.area())
        {
			// Add image and scene points from one view
            addPoints(imageCorners, objectCorners);
            successes++;
         }
        //cout<<successes<<endl;
        //Draw the corners
        cv::drawChessboardCorners(image, boardSize, imageCorners, found);
        cv::imshow("Corners on Chessboard", image);
        cv::waitKey(100);
    }

	return successes;
}

// Add scene points and corresponding image points
void CameraCalibrator::addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners)
{

	// 2D image points from one view
	imagePoints.push_back(imageCorners);          
	// corresponding 3D scene points
    objectPoints.push_back(objectCorners);
}


// Calibrate the camera
// returns the re-projection error
double CameraCalibrator::calibrate(cv::Size &imageSize)
{
	// undistorter must be reinitialized
	mustInitUndistort= true;

	// start calibration
	return 
     calibrateCamera(objectPoints,      // the 3D points
                    imagePoints,        // the image points
                    imageSize,          // image size
                    cameraMatrix,       // output camera matrix
                    distCoeffs,         // output distortion matrix
                    rvecs, tvecs,       // Rs, Ts
                    CV_CALIB_USE_INTRINSIC_GUESS);              // set options

}


cv::Mat CameraCalibrator::remap(const cv::Mat &image)
{

	cout<<distCoeffs.size();
	cv::Mat undistorted;
    if (mustInitUndistort)
    {
        // called once per calibration
		cv::initUndistortRectifyMap(
			cameraMatrix,  // computed camera matrix
            distCoeffs,    // computed distortion matrix
            cv::Mat(),     // optional rectification (none) 
			cv::Mat(),     // camera matrix to generate undistorted
			cv::Size(320,240),
            CV_32FC1,      // type of output map
            map1, map2);   // the x and y mapping functions
         mustInitUndistort= false;
		
    }
	// Apply mapping functions
    cv::remap(image, undistorted, map1, map2, cv::INTER_LINEAR); // interpolation type
	return undistorted;
}


// Set the calibration options
// 8radialCoeffEnabled should be true if 8 radial coefficients are required (5 is default)
// tangentialParamEnabled should be true if tangeantial distortion is present
void CameraCalibrator::setCalibrationFlag(bool radial8CoeffEnabled, bool tangentialParamEnabled)
{

    // Set the flag used in cv::calibrateCamera()
    flag = 0;
    if (!tangentialParamEnabled) flag += CV_CALIB_ZERO_TANGENT_DIST;
	if (radial8CoeffEnabled) flag += CV_CALIB_RATIONAL_MODEL;
}


void CameraCalibrator::get_Transform_matrix( std::vector<cv::Mat> &image_R_vector, std::vector<cv::Mat> &image_t_vector)
{

    image_R_vector=rvecs;
    image_t_vector=tvecs;
    for(int i=0;i<image_t_vector.size();i++)
       {
        cout<<"rvecs "<<i<<rvecs[i].t()<<endl;
        cout<<"tvecs "<<i<<tvecs[i].t()<<endl;
       }
}

