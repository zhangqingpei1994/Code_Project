#include"facemain.h"
#include <sys/types.h>
#include <sys/stat.h>
ostringstream stream1;
int initial=0;
int numbersele = 0;
int status=0;

//#include "dir.h"
//const char *faceCascadeFilename = "lbpcascade_frontalface.xml";     // LBP face detector.
//const char *faceCascadeFilename = "haarcascade_frontalface_alt_tree.xml";  // Haar face detector.
const char *faceCascadeFilename = "/home/zhang/brain_face_xml/haarcascade_frontalface_alt2.xml";
const char *eyeCascadeFilename1 = "/home/zhang/brain_face_xml/haarcascade_lefteye_2splits.xml";   // Best eye detector for open-or-closed eyes.
const char *eyeCascadeFilename2 = "/home/zhang/brain_face_xml/haarcascade_righteye_2splits.xml";   // Best eye detector for open-or-closed eyes.
//const char *eyeCascadeFilename1 = "haarcascade_mcs_lefteye.xml";       // Good eye detector for open-or-closed eyes.
//const char *eyeCascadeFilename2 = "haarcascade_mcs_righteye.xml";       // Good eye detector for open-or-closed eyes.
//const char *eyeCascadeFilename1 = "haarcascade_eye.xml";               // Basic eye detector for open eyes only.
//const char *eyeCascadeFilename2 = "haarcascade_eye_tree_eyeglasses.xml";
const char *facerecAlgorithm = "FaceRecognizer.Fisherfaces";
const int faceWidth = 70;
const int faceHeight = faceWidth;
const float UNKNOWN_PERSON_THRESHOLD = 0.7f;
ofstream outfile;

// Try to set the camera resolution. Note that this only works for some cameras on
// some computers and only for some drivers, so don't rely on it to work!
const int DESIRED_CAMERA_WIDTH = 640;
const int DESIRED_CAMERA_HEIGHT = 480;
#if !defined VK_ESCAPE
#define VK_ESCAPE 0x1B      // Escape character (27)
#endif

// Parameters controlling how often to keep new faces when collecting them. Otherwise, the training set could look to similar to each other!
const double CHANGE_IN_IMAGE_FOR_COLLECTION = 0.3;      // How much the facial image should change before collecting a new face photo for training.
const double CHANGE_IN_SECONDS_FOR_COLLECTION = 1.0;       // How much time must pass before collecting a new face photo for training.

const char *windowName = "WebcamFaceRec";   // Name shown in the GUI window.
const int BORDER = 8;  // Border between GUI elements to the edge of the image.

const bool preprocessLeftAndRightSeparately = true;   // Preprocess left & right sides of the face separately, in case there is stronger light on one side.
enum MODES { MODE_STARTUP = 0, MODE_DETECTION, MODE_COLLECT_FACES, MODE_TRAINING, MODE_RECOGNITION, MODE_DELETE_ALL, MODE_END };
const char* MODE_NAMES[] = { "Startup", "Detection", "Collect Faces", "Training", "Recognition", "Delete All", "ERROR!" };
MODES m_mode = MODE_STARTUP;
int m_selectedPerson = -1;
int m_numPersons = 0;
vector<int> m_latestFaces;

// Position of GUI buttons:
Rect m_rcBtnAdd;
Rect m_rcBtnDel;
Rect m_rcBtnDebug;
int m_gui_faces_left = -1;
int m_gui_faces_top = -1;

// Set to true if you want to see many windows created, showing various debug info. Set to 0 otherwise.
bool m_debug = false;
using namespace cv;
using namespace std;

template <typename T> string toString(T t)
{
    ostringstream out;
    out << t;
    return out.str();
}

template <typename T> T fromString(string t)
{
    T out;
    istringstream in(t);
    in >> out;
    return out;
}

void initDetectors(CascadeClassifier &faceCascade, CascadeClassifier &eyeCascade1, CascadeClassifier &eyeCascade2)
{
    // Load the Face Detection cascade classifier xml file.
    try {   // Surround the OpenCV call by a try/catch block so we can give a useful error message!
        faceCascade.load(faceCascadeFilename);

    }
    catch (cv::Exception &e) {}
    if (faceCascade.empty()) {
        cerr << "ERROR: Could not load Face Detection cascade classifier [" << faceCascadeFilename << "]!" << endl;
        cerr << "Copy the file from your OpenCV data folder (eg: 'C:\\OpenCV\\data\\lbpcascades') into this WebcamFaceRec folder." << endl;
        exit(1);
    }
    cout << "Loaded the Face Detection cascade classifier [" << faceCascadeFilename << "]." << endl;

    // Load the Eye Detection cascade classifier xml file.
    try {   // Surround the OpenCV call by a try/catch block so we can give a useful error message!
        eyeCascade1.load(eyeCascadeFilename1);
    }
    catch (cv::Exception &e) {}
    if (eyeCascade1.empty()) {
        cerr << "ERROR: Could not load 1st Eye Detection cascade classifier [" << eyeCascadeFilename1 << "]!" << endl;
        cerr << "Copy the file from your OpenCV data folder (eg: 'C:\\OpenCV\\data\\haarcascades') into this WebcamFaceRec folder." << endl;
        exit(1);
    }
    cout << "Loaded the 1st Eye Detection cascade classifier [" << eyeCascadeFilename1 << "]." << endl;

    // Load the Eye Detection cascade classifier xml file.
    try {   // Surround the OpenCV call by a try/catch block so we can give a useful error message!
        eyeCascade2.load(eyeCascadeFilename2);
    }
    catch (cv::Exception &e) {}
    if (eyeCascade2.empty()) {
        cerr << "Could not load 2nd Eye Detection cascade classifier [" << eyeCascadeFilename2 << "]." << endl;
        // Dont exit if the 2nd eye detector did not load, because we have the 1st eye detector at least.
        //exit(1);
    }
    else
        cout << "Loaded the 2nd Eye Detection cascade classifier [" << eyeCascadeFilename2 << "]." << endl;
}
Rect drawString(Mat img, string text, Point coord, Scalar color, float fontScale = 0.6f, int thickness = 1, int fontFace = FONT_HERSHEY_COMPLEX)
{
    // Get the text size & baseline.
    int baseline = 0;
    Size textSize = getTextSize(text, fontFace, fontScale, thickness, &baseline);
    baseline += thickness;

    // Adjust the coords for left/right-justified or top/bottom-justified.
    if (coord.y >= 0) {
        // Coordinates are for the top-left corner of the text from the top-left of the image, so move down by one row.
        coord.y += textSize.height;
    }
    else {
        // Coordinates are for the bottom-left corner of the text from the bottom-left of the image, so come up from the bottom.
        coord.y += img.rows - baseline + 1;
    }
    // Become right-justified if desired.
    if (coord.x < 0) {
        coord.x += img.cols - textSize.width + 1;
    }

    // Get the bounding box around the text.
    Rect boundingRect = Rect(coord.x, coord.y - textSize.height, textSize.width, baseline + textSize.height);

    // Draw anti-aliased text.
    putText(img, text, coord, fontFace, fontScale, color, thickness, CV_AA);

    // Let the user know how big their text is, in case they want to arrange things.
    return boundingRect;
}
Rect drawButton(Mat img, string text, Point coord, int minWidth = 0)
{
    int B = BORDER;
    Point textCoord = Point(coord.x + B, coord.y + B);
    // Get the bounding box around the text.
    Rect rcText = drawString(img, text, textCoord, CV_RGB(0, 0, 0));
    // Draw a filled rectangle around the text.
    Rect rcButton = Rect(rcText.x - B, rcText.y - B, rcText.width + 2 * B, rcText.height + 2 * B);
    // Set a minimum button width.
    if (rcButton.width < minWidth)
        rcButton.width = minWidth;
    // Make a semi-transparent white rectangle.
    Mat matButton = img(rcButton);
    matButton += CV_RGB(90, 90, 90);
    // Draw a non-transparent white border.
    rectangle(img, rcButton, CV_RGB(200, 200, 200), 1, CV_AA);

    // Draw the actual text that will be displayed, using anti-aliasing.
    drawString(img, text, textCoord, CV_RGB(10, 55, 20));

    return rcButton;
}
void initWebcam(VideoCapture &videoCapture, int cameraNumber)
{
    // Get access to the default camera.
    try {   // Surround the OpenCV call by a try/catch block so we can give a useful error message!
        videoCapture.open(cameraNumber);
    }
    catch (cv::Exception &e) {}//²¶×œÒì³£
    if (!videoCapture.isOpened()) {
        cerr << "ERROR: Could not access the camera!" << endl;
        exit(1);
    }
    cout << "Loaded camera " << cameraNumber << "." << endl;
}
bool isPointInRect(const Point pt, const Rect rc)
{
    if (pt.x >= rc.x && pt.x <= (rc.x + rc.width - 1))
    if (pt.y >= rc.y && pt.y <= (rc.y + rc.height - 1))
        return true;

    return false;
}
void onMouse(int event, int x, int y, int, void*)
{
    // We only care about left-mouse clicks, not right-mouse clicks or mouse movement.
    if (event != CV_EVENT_LBUTTONDOWN)// CV_EVENT_LBUTTONDOWN    =1,
        return;//Êó±ê²Ù×÷²»ÊÇ°ŽÏÂÊ±ŸÍ·µ»Ø
    // Check if the user clicked on one of our GUI buttons.
    Point pt = Point(x, y);
    if (isPointInRect(pt, m_rcBtnAdd))
    {//Œì²éµã»÷µÄÊÇ·ñŽŠÓÚÌíŒÓÈËÎï°ŽÅ¥
        cout << "User clicked [Add Person] button when numPersons was " << m_numPersons << endl;
        // Check if there is already a person without any collected faces, then use that person instead.
        // This can be checked by seeing if an image exists in their "latest collected face".
        if ((m_numPersons == 0) || (numbersele >=10))
        {
            // Add a new person.
            numbersele = 0;
            stream1.str("");
            if(m_numPersons == 0)
            stream1 << m_numPersons+initial+1;
            else
            stream1 << m_numPersons+initial;
            string filename;
            filename ="../facexml/"+stream1.str();
            status = 0;
            status = mkdir(filename.c_str(),S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
            m_numPersons++;
            status = m_numPersons + initial;
            outfile.open("../textxml/myfile.dat", std::ofstream::binary);
            outfile.write(reinterpret_cast<const char*>(&status), sizeof(int));
            outfile.close();
            cout << " Persons: " << status << endl;
            //cout << "Num Persons: " << m_numPersons+initial<< endl;

        }
        // Use the newly added person. Also use the newest person even if that person was empty.
        m_mode = MODE_COLLECT_FACES;
    }
    else if (isPointInRect(pt, m_rcBtnDel)) {
        cout << "User clicked [Delete All] button." << endl;
        m_mode = MODE_RECOGNITION;
    }
    else if (isPointInRect(pt, m_rcBtnDebug)) {
        cout << "User clicked [Debug] button." << endl;
        m_mode = MODE_RECOGNITION;
    }
    else
    {
        cout << "User clicked on the image and m_mode = MODE_TRAINING" << endl;
          m_mode = MODE_TRAINING;
        // Otherwise they clicked in the center.
    }
}
int onlyrecognized(Ptr<FaceRecognizer> &model,VideoCapture &videoCapture, CascadeClassifier &faceCascade, CascadeClassifier &eyeCascade1, CascadeClassifier &eyeCascade2)
{
    string outputStr;
    double similarity;
    // Run forever, until the user hits Escape to "break" out of this loop.
    while (true)
    {
        // Grab the next camera frame. Note that you can't modify camera frames.
        Mat cameraFrame;
        videoCapture >> cameraFrame;
        if (cameraFrame.empty())
        {
            cout << "ERROR: Couldn't grab the next camera frame." << endl;
            exit(1);
        }
        // Get a copy of the camera frame that we can draw onto.
        Mat displayedFrame;
        cameraFrame.copyTo(displayedFrame);
        // Run the face recognition system on the camera image. It will draw some things onto the given image, so make sure it is not read-only memory!
        int identity = -1;
        // Find a face and preprocess it to have a standard size and contrast & brightness.
        Rect faceRect;  // Position of detected face.
        Rect searchedLeftEye, searchedRightEye; // top-left and top-right regions of the face, where eyes were searched.
        Point leftEye, rightEye;    // Position of the detected eyes.
       Mat preprocessedFace = getPreprocessedFace(displayedFrame, faceWidth, faceCascade, eyeCascade1, eyeCascade2, preprocessLeftAndRightSeparately, &faceRect, &leftEye, &rightEye, &searchedLeftEye, &searchedRightEye);//Œì²âµœµÄÈËÁ³²¢ÇÒœøÐÐ·ÂÉä±ä»»Ö®ºó

        bool gotFaceAndEyes = false;
        if (preprocessedFace.data)
            gotFaceAndEyes = true;
        // Draw an anti-aliased rectangle around the detected face.
        if (faceRect.width > 0)
        {
            rectangle(displayedFrame, faceRect, CV_RGB(255, 255, 110), 2, CV_AA);
        // Draw light-blue anti-aliased circles for the 2 eyes.
        Scalar eyeColor = CV_RGB(0, 255, 255);
        if (leftEye.x >= 0) {   // Check if the eye was detected
            circle(displayedFrame, Point(faceRect.x + leftEye.x, faceRect.y + leftEye.y), 6, eyeColor, 1, CV_AA);
        }
        if (rightEye.x >= 0) {   // Check if the eye was detected
            circle(displayedFrame, Point(faceRect.x + rightEye.x, faceRect.y + rightEye.y), 6, eyeColor, 1, CV_AA);
        }
        }

            if (gotFaceAndEyes )
            {

                // Generate a face approximation by back-projecting the eigenvectors & eigenvalues.
                Mat reconstructedFace;
                reconstructedFace = reconstructFace(model, preprocessedFace);
                if (reconstructedFace.data)
                    //imshow("reconstructedFace", reconstructedFace);

                // Verify whether the reconstructed face looks like the preprocessed face, otherwise it is probably an unknown person.
                similarity = getSimilarity(preprocessedFace, reconstructedFace);

                if (similarity < UNKNOWN_PERSON_THRESHOLD)
                {
                    // Identify who the person is in the preprocessed face image.
                    identity = model->predict(preprocessedFace);
                    outputStr = toString(identity);
                     cout << "Identity: " << outputStr ;
                    return identity;

                   // break;
                }
                else {
                    // Since the confidence is low, assume it is an unknown person.
                    outputStr = "Unknown";
                }
                cout << "Identity: " << outputStr << ". Similarity: " << similarity << endl;

                // Show the confidence rating for the recognition in the mid-top of the display.
                int cx = (displayedFrame.cols - faceWidth) / 2;
                Point ptBottomRight = Point(cx - 5, BORDER + faceHeight);
                Point ptTopLeft = Point(cx - 15, BORDER);
                // Draw a gray line showing the threshold for an "unknown" person.
                Point ptThreshold = Point(ptTopLeft.x, ptBottomRight.y - (1.0 - UNKNOWN_PERSON_THRESHOLD) * faceHeight);
                rectangle(displayedFrame, ptThreshold, Point(ptBottomRight.x, ptThreshold.y), CV_RGB(200, 200, 200), 1, CV_AA);
                // Crop the confidence rating between 0.0 to 1.0, to show in the bar.
                double confidenceRatio = 1.0 - min(max(similarity, 0.0), 1.0);
                Point ptConfidence = Point(ptTopLeft.x, ptBottomRight.y - confidenceRatio * faceHeight);
                // Show the light-blue confidence bar.
                rectangle(displayedFrame, ptConfidence, ptBottomRight, CV_RGB(0, 255, 255), CV_FILLED, CV_AA);
                // Show the gray border of the bar.
                rectangle(displayedFrame, ptTopLeft, ptBottomRight, CV_RGB(200, 200, 200), 1, CV_AA);
            }


        // Show the current preprocessed face in the top-center of the display.
        int cx = (displayedFrame.cols - faceWidth) / 2;
        if (preprocessedFace.data) {
            // Get a BGR version of the face, since the output is BGR color.
            Mat srcBGR = Mat(preprocessedFace.size(), CV_8UC3);
            cvtColor(preprocessedFace, srcBGR, CV_GRAY2BGR);
            // Get the destination ROI (and make sure it is within the image!).
            //min(m_gui_faces_top + i * faceHeight, displayedFrame.rows - faceHeight);
            Rect dstRC = Rect(cx, BORDER, faceWidth, faceHeight);
            Mat dstROI = displayedFrame(dstRC);
            // Copy the pixels from src to dst.
            srcBGR.copyTo(dstROI);//ŽŠÀíºóµÄÍŒÏñœøÐÐÏÔÊŸ£¬²ÉÓÃBGR
        }
        // Draw an anti-aliased border around the face, even if it is not shown.
        rectangle(displayedFrame, Rect(cx - 1, BORDER - 1, faceWidth + 2, faceHeight + 2), CV_RGB(200, 200, 200), 1, CV_AA);

        // Draw the GUI buttons into the main image.
        m_rcBtnAdd = drawButton(displayedFrame, "Add Person", Point(BORDER, BORDER));
        m_rcBtnDel = drawButton(displayedFrame, "Delete All", Point(m_rcBtnAdd.x, m_rcBtnAdd.y + m_rcBtnAdd.height), m_rcBtnAdd.width);
        m_rcBtnDebug = drawButton(displayedFrame, "Debug", Point(m_rcBtnDel.x, m_rcBtnDel.y + m_rcBtnDel.height), m_rcBtnAdd.width);

        // Show the most recent face for each of the collected people, on the right side of the display.
        m_gui_faces_left = displayedFrame.cols - BORDER - faceWidth;
        m_gui_faces_top = BORDER;
        // Highlight the person being collected, using a red rectangle around their face.
        if (m_mode == MODE_COLLECT_FACES) {
            if (m_selectedPerson >= 0 && m_selectedPerson < m_numPersons) {
                int y = min(m_gui_faces_top + m_selectedPerson * faceHeight, displayedFrame.rows - faceHeight);
                Rect rc = Rect(m_gui_faces_left, y, faceWidth, faceHeight);
                rectangle(displayedFrame, rc, CV_RGB(255, 0, 0), 3, CV_AA);
            }
        }

        // Highlight the person that has been recognized, using a green rectangle around their face.
        if (identity >= 0 && identity < 1000) {
            int y = min(m_gui_faces_top + identity * faceHeight, displayedFrame.rows - faceHeight);
            Rect rc = Rect(m_gui_faces_left, y, faceWidth, faceHeight);
            rectangle(displayedFrame, rc, CV_RGB(0, 255, 0), 3, CV_AA);
        }

        // Show the camera frame on the screen.
        imshow(windowName, displayedFrame);

        // If the user wants all the debug data, show it to them!
//            if (!model.empty())
//                showTrainingDebugData(model, faceWidth, faceHeight);
        char keypress = waitKey(20);  // This is needed if you want to see anything!

        if (keypress == VK_ESCAPE)
        {   // Escape Key
            // Quit the program!
            break;
        }

    }//end while
}



void recognizeAndTrainUsingWebcam( Ptr<FaceRecognizer> &model,VideoCapture &videoCapture, CascadeClassifier &faceCascade, CascadeClassifier &eyeCascade1, CascadeClassifier &eyeCascade2)
{

    vector<Mat> preprocessedFaces;
    vector<int> faceLabels;
    Mat old_prepreprocessedFace;
    double old_time = 0;
    // Since we have already initialized everything, lets start in Detection mode.
    m_mode = MODE_DETECTION;//=1
    // Run forever, until the user hits Escape to "break" out of this loop.
    while (true)
    {

        // Grab the next camera frame. Note that you can't modify camera frames.
        Mat cameraFrame;
        videoCapture >> cameraFrame;
        if (cameraFrame.empty())
        {
            cerr << "ERROR: Couldn't grab the next camera frame." << endl;
            exit(1);
        }
        // Get a copy of the camera frame that we can draw onto.
        Mat displayedFrame;
        cameraFrame.copyTo(displayedFrame);
        // Run the face recognition system on the camera image. It will draw some things onto the given image, so make sure it is not read-only memory!
        int identity = -1;
        // Find a face and preprocess it to have a standard size and contrast & brightness.
        Rect faceRect;  // Position of detected face.
        Rect searchedLeftEye, searchedRightEye; // top-left and top-right regions of the face, where eyes were searched.
        Point leftEye, rightEye;    // Position of the detected eyes.
        Mat preprocessedFace = getPreprocessedFace(displayedFrame, faceWidth, faceCascade, eyeCascade1, eyeCascade2, preprocessLeftAndRightSeparately, &faceRect, &leftEye, &rightEye, &searchedLeftEye, &searchedRightEye);//Œì²âµœµÄÈËÁ³²¢ÇÒœøÐÐ·ÂÉä±ä»»Ö®ºó

        bool gotFaceAndEyes = false;
        if (preprocessedFace.data)
            gotFaceAndEyes = true;
        // Draw an anti-aliased rectangle around the detected face.
        if (faceRect.width > 0)
        {
            rectangle(displayedFrame, faceRect, CV_RGB(255, 255, 110), 2, CV_AA);
        // Draw light-blue anti-aliased circles for the 2 eyes.
        Scalar eyeColor = CV_RGB(0, 255, 255);
        if (leftEye.x >= 0) {   // Check if the eye was detected
            circle(displayedFrame, Point(faceRect.x + leftEye.x, faceRect.y + leftEye.y), 6, eyeColor, 1, CV_AA);
        }
        if (rightEye.x >= 0) {   // Check if the eye was detected
            circle(displayedFrame, Point(faceRect.x + rightEye.x, faceRect.y + rightEye.y), 6, eyeColor, 1, CV_AA);
        }
        }

        if (m_mode == MODE_DETECTION)
        {
            // Don't do anything special.
        }
        else if (m_mode == MODE_COLLECT_FACES)
        {
            // Check if we have detected a face.
            if (gotFaceAndEyes)
            {//Œì²âµœÈËÁ³Ê±²ÅÖŽÐÐ

                // Check if this face looks somewhat different from the previously collected face.
                double imageDiff = 10000000000.0;
                if (old_prepreprocessedFace.data)
                {
                    imageDiff = getSimilarity(preprocessedFace, old_prepreprocessedFace);//ÀÏµÄÍŒÏñºÍÐÂµÄÍŒÏñ²î±ð±ÈœÏŽóµÄ²Å²ÉŒ¯
                }

                // Also record when it happened.
                double current_time = (double)getTickCount();
                double timeDiff_seconds = (current_time - old_time) / getTickFrequency();
                // Only process the face if it is noticeably different from the previous frame and there has been noticeable time gap.
                if ((imageDiff > CHANGE_IN_IMAGE_FOR_COLLECTION) && (timeDiff_seconds > CHANGE_IN_SECONDS_FOR_COLLECTION))
                {// CHANGE_IN_IMAGE_FOR_COLLECTION=0.3ãÐÖµ  CHANGE_IN_SECONDS_FOR_COLLECTION=0.1Ò»Ãë²ÉŒ¯Ò»ÕÅ
                    // Also add the mirror image to the training set, so we have more training data, as well as to deal with faces looking to the left or right.
                    Mat mirroredFace;
                    flip(preprocessedFace, mirroredFace, 1);//Ë®Æœ·­×ª

                    // Add the face images to the list of detected faces.
                    preprocessedFaces.push_back(preprocessedFace);//preprocessedFaceÈËÁ³ÊÇŸ­¹ý·ÂÉä±ä»»ºÍÖ±·œÍŒÒÔºóµÄÁ³
                    preprocessedFaces.push_back(mirroredFace);
                    ostringstream stream;
                    stream << numbersele; //nÎªÒª×ª×Ö·ûŽ®µÄÕûÊý
                    string filename;
                    filename = "../facexml/" + stream1.str() + "/" + stream.str()+".jpg";
                    imwrite(filename.c_str(), displayedFrame);
                    faceLabels.push_back(status);//m_selectedPersonµÄÖµŽÓ0¿ªÊŒ
                    faceLabels.push_back(status);
                    numbersele++;
                    // Show the number of collected faces. But since we also store mirrored faces, just show how many the user thinks they stored.
                    cout << "Saved face " << (preprocessedFaces.size() / 2) << " for person " << status << endl;

                    // Make a white flash on the face, so the user knows a photo has been taken.
                    Mat displayedFaceRegion = displayedFrame(faceRect);
                    displayedFaceRegion += CV_RGB(90, 90, 90);

                    // Keep a copy of the processed face, to compare on next iteration.
                    old_prepreprocessedFace = preprocessedFace;
                    old_time = current_time;
                }
            }
        }
        else if (m_mode == MODE_TRAINING) {

            // Check if there is enough data to train from. For Eigenfaces, we can learn just one person if we want, but for Fisherfaces,
            // we need atleast 2 people otherwise it will crash!
            bool haveEnoughData = true;
            if (  preprocessedFaces.size() <= 18 || preprocessedFaces.size() != faceLabels.size()) {
                cout << "Warning: Need some training data before it can be learnt! Collect more data ..." << endl;
                haveEnoughData = false;
            }
            if (haveEnoughData) {
                // Start training from the collected faces using Eigenfaces or a similar algorithm.
                learnCollectedFaces(model,preprocessedFaces, faceLabels, facerecAlgorithm);//œøÐÐÑµÁ·
                // Now that training is over, we can start recognizing!
                m_mode = MODE_RECOGNITION;
                model->save("../facexml/Fisher_Model.xml");
            }
            else {
                // Since there isn't enough training data, go back to the face collection mode!
                m_mode = MODE_COLLECT_FACES;
            }

        }
        else if (m_mode == MODE_RECOGNITION) {
            if (gotFaceAndEyes ) {

                // Generate a face approximation by back-projecting the eigenvectors & eigenvalues.
                Mat reconstructedFace;
                reconstructedFace = reconstructFace(model, preprocessedFace);
                if (reconstructedFace.data)
                    imshow("reconstructedFace", reconstructedFace);

                // Verify whether the reconstructed face looks like the preprocessed face, otherwise it is probably an unknown person.
                double similarity = getSimilarity(preprocessedFace, reconstructedFace);

                string outputStr;
                if (similarity < UNKNOWN_PERSON_THRESHOLD) {
                    // Identify who the person is in the preprocessed face image.
                    identity = model->predict(preprocessedFace);
                    outputStr = toString(identity);
                }
                else {
                    // Since the confidence is low, assume it is an unknown person.
                    outputStr = "Unknown";
                }
                cout << "Identity: " << outputStr << ". Similarity: " << similarity << endl;

                // Show the confidence rating for the recognition in the mid-top of the display.
                int cx = (displayedFrame.cols - faceWidth) / 2;
                Point ptBottomRight = Point(cx - 5, BORDER + faceHeight);
                Point ptTopLeft = Point(cx - 15, BORDER);
                // Draw a gray line showing the threshold for an "unknown" person.
                Point ptThreshold = Point(ptTopLeft.x, ptBottomRight.y - (1.0 - UNKNOWN_PERSON_THRESHOLD) * faceHeight);
                rectangle(displayedFrame, ptThreshold, Point(ptBottomRight.x, ptThreshold.y), CV_RGB(200, 200, 200), 1, CV_AA);
                // Crop the confidence rating between 0.0 to 1.0, to show in the bar.
                double confidenceRatio = 1.0 - min(max(similarity, 0.0), 1.0);
                Point ptConfidence = Point(ptTopLeft.x, ptBottomRight.y - confidenceRatio * faceHeight);
                // Show the light-blue confidence bar.
                rectangle(displayedFrame, ptConfidence, ptBottomRight, CV_RGB(0, 255, 255), CV_FILLED, CV_AA);
                // Show the gray border of the bar.
                rectangle(displayedFrame, ptTopLeft, ptBottomRight, CV_RGB(200, 200, 200), 1, CV_AA);
            }
        }
        else if (m_mode == MODE_DELETE_ALL) {
            // Restart everything!
            m_selectedPerson = -1;
            m_numPersons = 0;
            m_latestFaces.clear();
            preprocessedFaces.clear();
            faceLabels.clear();
            old_prepreprocessedFace = Mat();

            // Restart in Detection mode.
            m_mode = MODE_DETECTION;
        }
        else {
            cerr << "ERROR: Invalid run mode " << m_mode << endl;
            exit(1);
        }

        // Show the help, while also showing the number of collected faces. Since we also collect mirrored faces, we should just
        // tell the user how many faces they think we saved (ignoring the mirrored faces), hence divide by 2.
        string help;
        Rect rcHelp;
        if (m_mode == MODE_DETECTION)
            help = "Click [Add Person] when ready to collect faces.";
        else if (m_mode == MODE_COLLECT_FACES)
            help = "Click anywhere to train from your " + toString(preprocessedFaces.size() / 2) + " faces of " + toString(m_numPersons) + " people.";
        else if (m_mode == MODE_TRAINING)
            help = "Please wait while your " + toString(preprocessedFaces.size() / 2) + " faces of " + toString(m_numPersons) + " people builds.";
        else if (m_mode == MODE_RECOGNITION)
            help = "Click people on the right to add more faces to them, or [Add Person] for someone new.";
        if (help.length() > 0) {
            // Draw it with a black background and then again with a white foreground.
            // Since BORDER may be 0 and we need a negative position, subtract 2 from the border so it is always negative.
            float txtSize = 0.4;
            drawString(displayedFrame, help, Point(BORDER, -BORDER - 2), CV_RGB(0, 0, 0), txtSize);  // Black shadow.
            rcHelp = drawString(displayedFrame, help, Point(BORDER + 1, -BORDER - 1), CV_RGB(255, 255, 255), txtSize);  // White text.
        }

        // Show the current mode.
        if (m_mode >= 0 && m_mode < MODE_END) {
            string modeStr = "MODE: " + string(MODE_NAMES[m_mode]);
            drawString(displayedFrame, modeStr, Point(BORDER, -BORDER - 2 - rcHelp.height), CV_RGB(0, 0, 0));       // Black shadow
            drawString(displayedFrame, modeStr, Point(BORDER + 1, -BORDER - 1 - rcHelp.height), CV_RGB(0, 255, 0)); // Green text
        }

        // Show the current preprocessed face in the top-center of the display.
        int cx = (displayedFrame.cols - faceWidth) / 2;
        if (preprocessedFace.data) {
            // Get a BGR version of the face, since the output is BGR color.
            Mat srcBGR = Mat(preprocessedFace.size(), CV_8UC3);
            cvtColor(preprocessedFace, srcBGR, CV_GRAY2BGR);
            // Get the destination ROI (and make sure it is within the image!).
            //min(m_gui_faces_top + i * faceHeight, displayedFrame.rows - faceHeight);
            Rect dstRC = Rect(cx, BORDER, faceWidth, faceHeight);
            Mat dstROI = displayedFrame(dstRC);
            // Copy the pixels from src to dst.
            srcBGR.copyTo(dstROI);//ŽŠÀíºóµÄÍŒÏñœøÐÐÏÔÊŸ£¬²ÉÓÃBGR
        }
        // Draw an anti-aliased border around the face, even if it is not shown.
        rectangle(displayedFrame, Rect(cx - 1, BORDER - 1, faceWidth + 2, faceHeight + 2), CV_RGB(200, 200, 200), 1, CV_AA);

        // Draw the GUI buttons into the main image.
        m_rcBtnAdd = drawButton(displayedFrame, "Add Person", Point(BORDER, BORDER));
        m_rcBtnDel = drawButton(displayedFrame, "Delete All", Point(m_rcBtnAdd.x, m_rcBtnAdd.y + m_rcBtnAdd.height), m_rcBtnAdd.width);
        m_rcBtnDebug = drawButton(displayedFrame, "Debug", Point(m_rcBtnDel.x, m_rcBtnDel.y + m_rcBtnDel.height), m_rcBtnAdd.width);

        // Show the most recent face for each of the collected people, on the right side of the display.
        m_gui_faces_left = displayedFrame.cols - BORDER - faceWidth;
        m_gui_faces_top = BORDER;
        // Highlight the person being collected, using a red rectangle around their face.
        if (m_mode == MODE_COLLECT_FACES) {
            if (m_selectedPerson >= 0 && m_selectedPerson < m_numPersons) {
                int y = min(m_gui_faces_top + m_selectedPerson * faceHeight, displayedFrame.rows - faceHeight);
                Rect rc = Rect(m_gui_faces_left, y, faceWidth, faceHeight);
                rectangle(displayedFrame, rc, CV_RGB(255, 0, 0), 3, CV_AA);
            }
        }

        // Highlight the person that has been recognized, using a green rectangle around their face.
        if (identity >= 0 && identity < 1000) {
            int y = min(m_gui_faces_top + identity * faceHeight, displayedFrame.rows - faceHeight);
            Rect rc = Rect(m_gui_faces_left, y, faceWidth, faceHeight);
            rectangle(displayedFrame, rc, CV_RGB(0, 255, 0), 3, CV_AA);
        }

        // Show the camera frame on the screen.
        imshow(windowName, displayedFrame);

        // If the user wants all the debug data, show it to them!
        if (m_debug) {
            Mat face;
            if (faceRect.width > 0) {
                face = cameraFrame(faceRect);
                if (searchedLeftEye.width > 0 && searchedRightEye.width > 0) {
                    Mat topLeftOfFace = face(searchedLeftEye);
                    Mat topRightOfFace = face(searchedRightEye);
                    imshow("topLeftOfFace", topLeftOfFace);
                    imshow("topRightOfFace", topRightOfFace);
                }
            }

            if (!model.empty())
                showTrainingDebugData(model, faceWidth, faceHeight);
        }


        // IMPORTANT: Wait for atleast 20 milliseconds, so that the image can be displayed on the screen!
        // Also checks if a key was pressed in the GUI window. Note that it should be a "char" to support Linux.
        char keypress = waitKey(20);  // This is needed if you want to see anything!

        if (keypress == VK_ESCAPE) {   // Escape Key
            // Quit the program!
            break;
        }

    }//end while
}

int recognized()
{
    CascadeClassifier faceCascade;
    CascadeClassifier eyeCascade1;
    CascadeClassifier eyeCascade2;
    VideoCapture videoCapture;
    Ptr<FaceRecognizer>model;
     model = Algorithm::create<FaceRecognizer>(facerecAlgorithm);
    ifstream inF;
    inF.open("/home/zhang/brain_face_xml/myfile.dat", std::ifstream::binary);
    inF.read(reinterpret_cast<char*>(&initial), sizeof(int));
    inF.close();
    model->load("/home/zhang/brain_face_xml/Fisher_Model.xml");
    //initial = m_numPersons;
    cout << "WebcamFaceRec, by Shervin Emami (www.shervinemami.info), June 2012." << endl;
    cout << "Realtime face detection + face recognition from a webcam using LBP and Eigenfaces or Fisherfaces." << endl;
    cout << "Compiled with OpenCV version " << CV_VERSION << endl << endl;
    //waitKey(5000);
    // Load the face and 1 or 2 eye detection XML classifiers.
    initDetectors(faceCascade, eyeCascade1, eyeCascade2);//ŒÓÔØž÷Àà·ÖÀàÆ÷

    cout << endl;
    cout << "Hit 'Escape' in the GUI window to quit." << endl;

    // Allow the user to specify a camera number, since not all computers will be the same camera number.
    int cameraNumber = 0;   // Change this if you want to use a different camera device.

    // Get access to the webcam.
    initWebcam(videoCapture, cameraNumber);//Žò¿ªÉãÏñÍ·

    // Try to set the camera resolution. Note that this only works for some cameras on
    // some computers and only for some drivers, so don't rely on it to work!
    videoCapture.set(CV_CAP_PROP_FRAME_WIDTH, DESIRED_CAMERA_WIDTH);//set·œ·š²»œöÓÃÓÚÈ¡ÊÓÆµÖ¡µÄÎ»ÖÃ£¬»¹¿ÉÒÔÉèÖÃÊÓÆµµÄÖ¡ÂÊ¡¢ÁÁ¶È
    videoCapture.set(CV_CAP_PROP_FRAME_HEIGHT, DESIRED_CAMERA_HEIGHT);//ÉèÖÃÍŒÏñµÄŽóÐ¡640X480

    // Create a GUI window for display on the screen.
    namedWindow(windowName); // Resizable window, might not work on Windows.
    // Get OpenCV to automatically call my "onMouse()" function when the user clicks in the GUI window.
    setMouseCallback(windowName, onMouse, 0);//ÉèÖÃÊó±êµÄÏìÓŠº¯Êý
    //waitKey(500);
    // Run Face Recogintion interactively from the webcam. This function runs until the user quits.
     return onlyrecognized(model,videoCapture, faceCascade, eyeCascade1, eyeCascade2);
     // cvMoveWindow("windowName",640,480);
      //destroyWindow("windowName");
     //destroyAllWindows();
}
void facebegin()
{
    CascadeClassifier faceCascade;
    CascadeClassifier eyeCascade1;
    CascadeClassifier eyeCascade2;
    VideoCapture videoCapture;
    Ptr<FaceRecognizer>model;
     model = Algorithm::create<FaceRecognizer>(facerecAlgorithm);
    ifstream inF;
    inF.open("/home/zhang/brain_face_xml/myfile.dat", std::ifstream::binary);
    inF.read(reinterpret_cast<char*>(&initial), sizeof(int));
    inF.close();
    model->load("/home/zhang/brain_face_xml/Fisher_Model.xml");
    //initial = m_numPersons;
    //waitKey(5000);
    cout << "WebcamFaceRec, by Shervin Emami (www.shervinemami.info), June 2012." << endl;
    cout << "Realtime face detection + face recognition from a webcam using LBP and Eigenfaces or Fisherfaces." << endl;
    cout << "Compiled with OpenCV version " << CV_VERSION << endl << endl;

    // Load the face and 1 or 2 eye detection XML classifiers.
    initDetectors(faceCascade, eyeCascade1, eyeCascade2);//ŒÓÔØž÷Àà·ÖÀàÆ÷

    cout << endl;
    cout << "Hit 'Escape' in the GUI window to quit." << endl;

    // Allow the user to specify a camera number, since not all computers will be the same camera number.
    int cameraNumber = 0;   // Change this if you want to use a different camera device.

    // Get access to the webcam.
    initWebcam(videoCapture, cameraNumber);//Žò¿ªÉãÏñÍ·

    // Try to set the camera resolution. Note that this only works for some cameras on
    // some computers and only for some drivers, so don't rely on it to work!
    videoCapture.set(CV_CAP_PROP_FRAME_WIDTH, DESIRED_CAMERA_WIDTH);//set·œ·š²»œöÓÃÓÚÈ¡ÊÓÆµÖ¡µÄÎ»ÖÃ£¬»¹¿ÉÒÔÉèÖÃÊÓÆµµÄÖ¡ÂÊ¡¢ÁÁ¶È
    videoCapture.set(CV_CAP_PROP_FRAME_HEIGHT, DESIRED_CAMERA_HEIGHT);//ÉèÖÃÍŒÏñµÄŽóÐ¡640X480

    // Create a GUI window for display on the screen.
    namedWindow(windowName); // Resizable window, might not work on Windows.
    // Get OpenCV to automatically call my "onMouse()" function when the user clicks in the GUI window.
    setMouseCallback(windowName, onMouse, 0);//ÉèÖÃÊó±êµÄÏìÓŠº¯Êý
    // waitKey(500);
    // Run Face Recogintion interactively from the webcam. This function runs until the user quits.
    recognizeAndTrainUsingWebcam(model,videoCapture, faceCascade, eyeCascade1, eyeCascade2);
     //destroyWindow("windowName");

}

