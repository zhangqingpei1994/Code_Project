#include"facerecognition.h"
void learnCollectedFaces(Ptr<FaceRecognizer> &model,const vector<Mat> preprocessedFaces, const vector<int> faceLabels, const string facerecAlgorithm)
{


    cout << "Learning the collected faces using the [" << facerecAlgorithm << "] algorithm ..." << endl;

    // Make sure the "contrib" module is dynamically loaded at runtime.
    // Requires OpenCV v2.4.1 or later (from June 2012), otherwise the FaceRecognizer will not compile or run!
    //bool haveContribModule = cvinitModule_contrib();
    //if (!haveContribModule) {
    //	cerr << "ERROR: The 'contrib' module is needed for FaceRecognizer but has not been loaded into OpenCV!" << endl;
    //	exit(1);
    //}

    // Use the new FaceRecognizer class in OpenCV's "contrib" module:
    // Requires OpenCV v2.4.1 or later (from June 2012), otherwise the FaceRecognizer will not compile or run!
    //model = createFisherFaceRecognizer();//ÈýÖÖÑ¡ÔñcreateEigenFaceRecognizer¡¢createFisherFaceRecognizer¡¢createLBPHFaceRecognizer¡£
    if (model.empty()) {
        cerr << "ERROR: The FaceRecognizer algorithm [" << facerecAlgorithm << "] is not available in your version of OpenCV. Please update to OpenCV v2.4.1 or newer." << endl;
        exit(1);
    }

    // Do the actual training from the collected faces. Might take several seconds or minutes depending on input!
    model->train(preprocessedFaces, faceLabels);


}
Mat getImageFrom1DFloatMat(const Mat matrixRow, int height)
{
    // Make it a rectangular shaped image instead of a single row.
    Mat rectangularMat = matrixRow.reshape(1, height);
    // Scale the values to be between 0 to 255 and store them as a regular 8-bit uchar image.
    Mat dst;
    normalize(rectangularMat, dst, 0, 255, NORM_MINMAX, CV_8UC1);
    return dst;
}
void showTrainingDebugData(const Ptr<FaceRecognizer> model, const int faceWidth, const int faceHeight)
{
    try {   // Surround the OpenCV calls by a try/catch block so we don't crash if some model parameters aren't available.

        // Show the average face (statistical average for each pixel in the collected images).
        Mat averageFaceRow = model->get<Mat>("mean");
        //printMatInfo(averageFaceRow, "averageFaceRow");
        // Convert the matrix row (1D float matrix) to a regular 8-bit image.
        Mat averageFace = getImageFrom1DFloatMat(averageFaceRow, faceHeight);
       // printMatInfo(averageFace, "averageFace");
       // imshow("averageFace", averageFace);

        // Get the eigenvectors
        Mat eigenvectors = model->get<Mat>("eigenvectors");
        //printMatInfo(eigenvectors, "eigenvectors");

        // Show the best 20 eigenfaces
        for (int i = 0; i < min(20, eigenvectors.cols); i++) {
            // Create a column vector from eigenvector #i.
            // Note that clone() ensures it will be continuous, so we can treat it like an array, otherwise we can't reshape it to a rectangle.
            // Note that the FaceRecognizer class already gives us L2 normalized eigenvectors, so we don't have to normalize them ourselves.
            Mat eigenvectorColumn = eigenvectors.col(i).clone();
            //printMatInfo(eigenvectorColumn, "eigenvector");

            Mat eigenface = getImageFrom1DFloatMat(eigenvectorColumn, faceHeight);
            //printMatInfo(eigenface, "eigenface");
            imshow(format("Eigenface%d", i), eigenface);
        }

        // Get the eigenvalues
        Mat eigenvalues = model->get<Mat>("eigenvalues");
        //printMat(eigenvalues, "eigenvalues");

        //int ncomponents = model->get<int>("ncomponents");
        //cout << "ncomponents = " << ncomponents << endl;

//        vector<Mat> projections = model->getProjections();
//        cout << "projections: " << projections.size() << endl;
//        for (int i = 0; i < (int)projections.size(); i++) {
//            printMat(projections[i], "projections");
//        }

        //labels = model->get<Mat>("labels");
        //printMat(labels, "labels");

    }
    catch (cv::Exception e) {
        //cout << "WARNING: Missing FaceRecognizer properties." << endl;
    }

}
Mat reconstructFace(const Ptr<FaceRecognizer> &model, const Mat &preprocessedFace)
{
    // Since we can only reconstruct the face for some types of FaceRecognizer models (ie: Eigenfaces or Fisherfaces),
    // we should surround the OpenCV calls by a try/catch block so we don't crash for other models.
    try {

        // Get some required data from the FaceRecognizer model
        //Ptr<FaceRecognizer> model1
        //model1 = createFisherFaceRecognizer();
        Mat eigenvectors = model->get<Mat>("eigenvectors");//»ñÈ¡ÌØÕ÷ÏòÁ¿
        Mat averageFaceRow = model->get<Mat>("mean");//»ñÈ¡ÆœŸùÈËÁ³

        int faceHeight = preprocessedFace.rows;
      // Mat reconstructedFace;
        // Project the input image onto the PCA subspace.
        //Mat projection = subspaceProject(eigenvectors, averageFaceRow, preprocessedFace.reshape(1, 1));
        //printMatInfo(projection, "projection");
        //if(faceHeight==4900)
       // {
       Mat projection = subspaceProject(eigenvectors, averageFaceRow, preprocessedFace.reshape(1, 1));
        // Generate the reconstructed face back from the PCA subspace.
        Mat reconstructionRow = subspaceReconstruct(eigenvectors, averageFaceRow, projection);
        //printMatInfo(reconstructionRow, "reconstructionRow");

        // Convert the float row matrix to a regular 8-bit image. Note that we
        // shouldn't use "getImageFrom1DFloatMat()" because we don't want to normalize
        // the data since it is already at the perfect scale.

        // Make it a rectangular shaped image instead of a single row.
        Mat reconstructionMat = reconstructionRow.reshape(1, faceHeight);
        // Convert the floating-point pixels to regular 8-bit uchar pixels.
        Mat reconstructedFace = Mat(reconstructionMat.size(), CV_8U);
        reconstructionMat.convertTo(reconstructedFace, CV_8U, 1, 0);
        //printMatInfo(reconstructedFace, "reconstructedFace");

           // Mat reconstructedFace=Mat(preprocessedFace.size(), CV_8U, Scalar(128));;

        return reconstructedFace;

    }
    catch (cv::Exception e) {
        //cout << "WARNING: Missing FaceRecognizer properties." << endl;
        return Mat();
    }
}

double getSimilarity(const Mat A, const Mat B)
{
    if (A.rows > 0 && A.rows == B.rows && A.cols > 0 && A.cols == B.cols) {
        // Calculate the L2 relative error between the 2 images.
        double errorL2 = norm(A, B, CV_L2);
        // Convert to a reasonable scale, since L2 error is summed across all pixels of the image.
        double similarity = errorL2 / (double)(A.rows * A.cols);//²éÖÆ¶ÈÆœ·œ¿ªžùºÅ
        return similarity;
    }
    else {
        //cout << "WARNING: Images have a different size in 'getSimilarity()'." << endl;
        return 100000000.0;  // Return a bad value
    }
}
