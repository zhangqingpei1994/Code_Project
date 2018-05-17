#include"bp.h"
using namespace cv;
void bp_train()
{
     int matRows = 0;
     int matCols = 6;
     Mat matData;
     Mat trainClasses1;
     string fileName = string("../bp_nn/joint.txt");
             ifstream inFile(fileName.c_str(), ios_base::in);
             if (!inFile.is_open())
             {
                 cout << "打开失败" << endl;


             }
       istream_iterator<float> begin(inFile);
       istream_iterator<float> end;
        vector<float> inData(begin, end);
        Mat tmpMat = Mat(inData);
        size_t dataLength = inData.size();
        matRows = dataLength / matCols;
        //3.ÊýŸÝ×Ü³€¶È
   if (dataLength != (matRows * matCols))
                {
                    cout << " 行和列不对" << endl;
                    matRows = dataLength;
                }

                // œ«ÎÄŒþÊýŸÝ±£ŽæÖÁÊä³öŸØÕó
                matData = tmpMat.reshape(0, matRows).clone();
                //cout << matData << endl;
                inFile.close();
                fileName = string("/home/linzc/QT/2-4/untitled_test/bp_nn/force.txt");
                ifstream inFile1(fileName.c_str(), ios_base::in);
                if (!inFile1.is_open())
                {
                    cout << "打开失败" << endl;

                }

                // ÔØÈëÊýŸÝ
                istream_iterator<float> begin1(inFile1);    //°Ž float žñÊœÈ¡ÎÄŒþÊýŸÝÁ÷µÄÆðÊŒÖžÕë
                istream_iterator<float> end1;          //È¡ÎÄŒþÁ÷µÄÖÕÖ¹Î»ÖÃ
                vector<float> inData1(begin1, end1);      //œ«ÎÄŒþÊýŸÝ±£ŽæÖÁ std::vector ÖÐ
                tmpMat = Mat(inData1);
                dataLength = inData.size();
                //2.ÐÐÁÐÊý
                matRows = dataLength / matCols;
                //3.ÊýŸÝ×Ü³€¶È
                if (dataLength != (matRows * matCols))
                {
                    cout << "行和列不对" << endl;


                    matRows = dataLength;
                }

                // œ«ÎÄŒþÊýŸÝ±£ŽæÖÁÊä³öŸØÕó
                trainClasses1 = 1.63*tmpMat.reshape(0, matRows).clone();
                //cout << trainClasses1 << endl;
                inFile1.close();
  CvANN_MLP bp;
  CvANN_MLP_TrainParams params;
  params.train_method=CvANN_MLP_TrainParams::BACKPROP;
   params.term_crit = cvTermCriteria(CV_TERMCRIT_EPS,5000,1e-5);
  // params.bp_dw_scale=0.1;
    Mat layerSizes=(Mat_<int>(1,3) << 6,17,6);
   bp.create(layerSizes,CvANN_MLP::SIGMOID_SYM);
   Mat weights(1, matData.rows, CV_32FC1, Scalar::all(1));
   bp.train(matData, trainClasses1, weights,Mat(), params);
   Mat sample(4, 6,CV_32FC1);
           sample.at<float>(0, 0) = -85.9176;
           sample.at<float>(0, 1) = -90.5799;
           sample.at<float>(0, 2) = -99.2203;
           sample.at<float>(0, 3) = -69.5154;
           sample.at<float>(0, 4) = 77.3906;
           sample.at<float>(0, 5) = -25.9873;

           sample.at<float>(1, 0) = -78.283;
           sample.at<float>(1, 1) = -78.4797;
           sample.at<float>(1, 2) = -106.471;
           sample.at<float>(1, 3) = -23.3328;
           sample.at<float>(1, 4) = 91.1467;
           sample.at<float>(1, 5) = -66.0451;

           sample.at<float>(2, 0) = -85.9203;
           sample.at<float>(2, 1) = -90.5757;
           sample.at<float>(2, 2) = -99.2189;
           sample.at<float>(2, 3) = -76.1869;
           sample.at<float>(2, 4) = 87.8138;
           sample.at<float>(2, 5) = -49.5508;

           sample.at<float>(2, 0) = -85.9176;
           sample.at<float>(2, 1) = -90.5792;
           sample.at<float>(2, 2) = -99.2203;
           sample.at<float>(2, 3) = -67.5474;
           sample.at<float>(2, 4) = 48.299;
           sample.at<float>(2, 5) = -49.5508;

           sample.at<float>(3, 0) = -85.9162;
           sample.at<float>(3, 1) = -90.5764;
           sample.at<float>(3, 2) = -99.2203;
           sample.at<float>(3, 3) = -33.664;
           sample.at<float>(3, 4) = 87.8104;
           sample.at<float>(3, 5) = -49.546;
       //Mat response(117, 6, CV_32FC1);
           Mat response;
           bp.predict(sample, response);
           bp.save("../untitled_test_2/textxml/bp_param.xml");
           response = response / 1.63;
           cout << response << endl;
}
