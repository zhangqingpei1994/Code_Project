/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */
 
#include "MainController.h"
using namespace std;

MainController::MainController(int argc, char * argv[])
 : good(true),
   eFusion(0),
   gui(0),
   groundTruthOdometry(0),
   logReader(0),
   framesToSkip(0),                     //个人理解是相机跳过的帧数
   resetButton(false),
   resizeStream(0)
{
    std::string empty;
    iclnuim = Parse::get().arg(argc, argv, "-icl", empty) > -1;   //false  Enable this if using the ICL-NUIM dataset (flips normals to account for negative focal length on that data).

    std::string calibrationFile;
    Parse::get().arg(argc, argv, "-cal", calibrationFile);

    //Resolution::getInstance(640, 480);
    Resolution::getInstance(512, 424);

    if(calibrationFile.length())     //获取相机标定参数
    {
        loadCalibration(calibrationFile);
    }
    else
    {
        //Intrinsics::getInstance(528, 528, 320, 240);
        Intrinsics::getInstance(364.531494, 364.531494, 256.237396, 207.297302);
    }

    Parse::get().arg(argc, argv, "-l", logFile);  //找到-l后面的文件名称，并赋值给logfile

    if(logFile.length())
    {
        logReader = new RawLogReader(logFile, Parse::get().arg(argc, argv, "-f", empty) > -1);  //传进来的第二个参数是false
    }
    else
    {
        bool flipColors = Parse::get().arg(argc,argv,"-f",empty) > -1;  //false  Flip RGB/BGR
        logReader = new LiveLogReader(logFile, flipColors, LiveLogReader::CameraType::OpenNI2);  //基类指针指向派生类对象

        good = ((LiveLogReader *)logReader)->cam->ok();

#ifdef WITH_REALSENSE
        if(!good)
        {
          delete logReader;
          logReader = new LiveLogReader(logFile, flipColors, LiveLogReader::CameraType::RealSense);

          good = ((LiveLogReader *)logReader)->cam->ok();
        }
#endif
    }

    if(Parse::get().arg(argc, argv, "-p", poseFile) > 0)      //Loads ground truth poses to use instead of estimated pose.
    {
        groundTruthOdometry = new GroundTruthOdometry(poseFile);
    }

    confidence = 10.0f;
    depth = 3.0f;
    icp = 10.0f;
    icpErrThresh = 5e-05;
    covThresh = 1e-05;     //Local loop closure covariance threshold
    photoThresh = 115;
    fernThresh = 0.3095f;

    timeDelta = 200;
    icpCountThresh = 40000;
    start = 1;
    so3 = !(Parse::get().arg(argc, argv, "-nso", empty) > -1);
    end = std::numeric_limits<unsigned short>::max(); //Funny bound, since we predict times in this format really! 65535

    Parse::get().arg(argc, argv, "-c", confidence);  //获得参数的值
    Parse::get().arg(argc, argv, "-d", depth);   // Cutoff distance for depth processing 深度处理的截至距离
    Parse::get().arg(argc, argv, "-i", icp);
    Parse::get().arg(argc, argv, "-ie", icpErrThresh);
    Parse::get().arg(argc, argv, "-cv", covThresh);
    Parse::get().arg(argc, argv, "-pt", photoThresh);
    Parse::get().arg(argc, argv, "-ft", fernThresh);
    Parse::get().arg(argc, argv, "-t", timeDelta);
    Parse::get().arg(argc, argv, "-ic", icpCountThresh);  // Local loop closure inlier threshold
    Parse::get().arg(argc, argv, "-s", start);
    Parse::get().arg(argc, argv, "-e", end);

    logReader->flipColors = Parse::get().arg(argc, argv, "-f", empty) > -1;

    //bool类型的数据
    openLoop = !groundTruthOdometry && Parse::get().arg(argc, argv, "-o", empty) > -1; //-o  Open loop mode.
    reloc = Parse::get().arg(argc, argv, "-rl", empty) > -1;      //enable relocalisation
    frameskip = Parse::get().arg(argc, argv, "-fs", empty) > -1;  // Frame skip if processing a log to simulate real-time.
    quiet = Parse::get().arg(argc, argv, "-q", empty) > -1;       //Quit when finished a log.
    fastOdom = Parse::get().arg(argc, argv, "-fo", empty) > -1;   //Fast odometry (single level pyramid).
    rewind = Parse::get().arg(argc, argv, "-r", empty) > -1;      //Rewind and loop log forever.
    frameToFrameRGB = Parse::get().arg(argc, argv, "-ftf", empty) > -1; //Do frame-to-frame RGB tracking.

    gui = new GUI(logFile.length() == 0, Parse::get().arg(argc, argv, "-sc", empty) > -1);   //Showcase mode (minimal GUI).

    gui->flipColors->Ref().Set(logReader->flipColors);
    gui->rgbOnly->Ref().Set(false);
    gui->pyramid->Ref().Set(true);
    gui->fastOdom->Ref().Set(fastOdom);
    gui->confidenceThreshold->Ref().Set(confidence);
    gui->depthCutoff->Ref().Set(depth);
    gui->icpWeight->Ref().Set(icp);
    gui->so3->Ref().Set(so3);
    gui->frameToFrameRGB->Ref().Set(frameToFrameRGB);

    resizeStream = new Resize(Resolution::getInstance().width(),
                              Resolution::getInstance().height(),
                              Resolution::getInstance().width() / 2,
                              Resolution::getInstance().height() / 2);
}

MainController::~MainController()
{
    if(eFusion)
    {
        delete eFusion;
    }

    if(gui)
    {
        delete gui;
    }

    if(groundTruthOdometry)
    {
        delete groundTruthOdometry;
    }

    if(logReader)
    {
        delete logReader;
    }

    if(resizeStream)
    {
        delete resizeStream;
    }
}

void MainController::loadCalibration(const std::string & filename)
{
    std::ifstream file(filename);
    std::string line;

    assert(!file.eof());

    double fx, fy, cx, cy;

    std::getline(file, line);

    int n = sscanf(line.c_str(), "%lg %lg %lg %lg", &fx, &fy, &cx, &cy);

    assert(n == 4 && "Ooops, your calibration file should contain a single line with fx fy cx cy!");

    Intrinsics::getInstance(fx, fy, cx, cy);
}

void MainController::launch()
{
    while(good)    //true
    {
        if(eFusion)  //0
        {
            run();
        }

        if(eFusion == 0 || resetButton) //resetButton=false
        {
            resetButton = false;

            if(eFusion)
            {
                delete eFusion;
            }

            logReader->rewind();
            eFusion = new ElasticFusion(openLoop ? std::numeric_limits<int>::max() / 2 : timeDelta,
                                        icpCountThresh,
                                        icpErrThresh,
                                        covThresh,
                                        !openLoop,
                                        iclnuim,
                                        reloc,
                                        photoThresh,     //Global loop closure photometric threshold (default 115).
                                        confidence,      //Surfel confidence threshold
                                        depth,           //最大有效深度
                                        icp,             //Relative ICP/RGB tracking weight
                                        fastOdom,
                                        fernThresh,
                                        so3,
                                        frameToFrameRGB, //Do frame-to-frame RGB tracking.
                                        logReader->getFile());
        }
        else
        {
            break;
        }

    }
}

void MainController::run()
{
    while(!pangolin::ShouldQuit() && !((!logReader->hasMore()) && quiet) && !(eFusion->getTick() == end && quiet))
    {
        if(!gui->pause->Get() || pangolin::Pushed(*gui->step))
        {
            if((logReader->hasMore() || rewind) && eFusion->getTick() < end)  //end是最大值,rewind=false
            {
                TICK("LogRead");
                if(rewind)    //不执行
                {
                    if(!logReader->hasMore())
                    {
                        logReader->getBack();
                    }
                    else
                    {
                        logReader->getNext();
                    }

                    if(logReader->rewound())
                    {
                        logReader->currentFrame = 0;
                    }
                }
                else       //执行
                {
                    logReader->getNext();       //====================================================================================================================================================
                }
                TOCK("LogRead");

                if(eFusion->getTick() < start)             //不执行  getTick 单调递增的函数
                {
                    eFusion->setTick(start);

                    logReader->fastForward(start);        //空函数
                }

                float weightMultiplier = framesToSkip + 1;  //正常情况下framesToSkip一直是0;
                //cout<<"framesToSkip="<<framesToSkip<<endl;

                if(framesToSkip > 0)  //正常不执行
                {
                    eFusion->setTick(eFusion->getTick() + framesToSkip);
                    logReader->fastForward(logReader->currentFrame + framesToSkip);
                    framesToSkip = 0;

                }

                Eigen::Matrix4f * currentPose = 0;

                if(groundTruthOdometry)    //不执行 这个执行的时候是Loads ground truth poses to use instead of estimated pose.
                {
                    currentPose = new Eigen::Matrix4f;
                    currentPose->setIdentity();
                    *currentPose = groundTruthOdometry->getTransformation(logReader->timestamp);
                }
                //weightMultiplier=1
                eFusion->processFrame(logReader->rgb, logReader->depth, logReader->timestamp, currentPose, weightMultiplier);    //====================================================================================

                if(currentPose)           //正常不执行
                {
                    delete currentPose;
                }

                if(frameskip && Stopwatch::getInstance().getTimings().at("Run") > 1000.f / 30.f)  //不执行正常
                {
                    framesToSkip = int(Stopwatch::getInstance().getTimings().at("Run") / (1000.f / 30.f));
                }
            }
        }
        else
        {
            eFusion->predict();
        }

        TICK("GUI");

        if(gui->followPose->Get())      //默认执行，但是好像核心的东西不是太多
        {
            pangolin::OpenGlMatrix mv;

            Eigen::Matrix4f currPose = eFusion->getCurrPose();
            Eigen::Matrix3f currRot = currPose.topLeftCorner(3, 3);

            Eigen::Quaternionf currQuat(currRot);
            Eigen::Vector3f forwardVector(0, 0, 1);
            Eigen::Vector3f upVector(0, iclnuim ? 1 : -1, 0);

            Eigen::Vector3f forward = (currQuat * forwardVector).normalized();
            Eigen::Vector3f up = (currQuat * upVector).normalized();

            Eigen::Vector3f eye(currPose(0, 3), currPose(1, 3), currPose(2, 3));

            eye -= forward;

            Eigen::Vector3f at = eye + forward;

            Eigen::Vector3f z = (eye - at).normalized();  // Forward
            Eigen::Vector3f x = up.cross(z).normalized(); // Right
            Eigen::Vector3f y = z.cross(x);

            Eigen::Matrix4d m;
            m << x(0),  x(1),  x(2),  -(x.dot(eye)),
                 y(0),  y(1),  y(2),  -(y.dot(eye)),
                 z(0),  z(1),  z(2),  -(z.dot(eye)),
                    0,     0,     0,              1;

            memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));

            gui->s_cam.SetModelViewMatrix(mv);
        }

        gui->preCall();

        std::stringstream stri;
        stri << eFusion->getModelToModel().lastICPCount;
        gui->trackInliers->Ref().Set(stri.str());

        std::stringstream stre;
        stre << (std::isnan(eFusion->getModelToModel().lastICPError) ? 0 : eFusion->getModelToModel().lastICPError);
        gui->trackRes->Ref().Set(stre.str());

        if(!gui->pause->Get())   //执行
        {
            gui->resLog.Log((std::isnan(eFusion->getModelToModel().lastICPError) ? std::numeric_limits<float>::max() : eFusion->getModelToModel().lastICPError), icpErrThresh);
            gui->inLog.Log(eFusion->getModelToModel().lastICPCount, icpCountThresh);
        }

        Eigen::Matrix4f pose = eFusion->getCurrPose();

        if(gui->drawRawCloud->Get() || gui->drawFilteredCloud->Get())  //不执行
        {
            eFusion->computeFeedbackBuffers();
        }

        if(gui->drawRawCloud->Get())      //不执行
        {
            eFusion->getFeedbackBuffers().at(FeedbackBuffer::RAW)->render(gui->s_cam.GetProjectionModelViewMatrix(), pose, gui->drawNormals->Get(), gui->drawColors->Get());
        }

        if(gui->drawFilteredCloud->Get()) //不执行
        {
            eFusion->getFeedbackBuffers().at(FeedbackBuffer::FILTERED)->render(gui->s_cam.GetProjectionModelViewMatrix(), pose, gui->drawNormals->Get(), gui->drawColors->Get());
        }

        if(gui->drawGlobalModel->Get())                                 //执行
        {
            glFinish();            //用于向图形硬件提交缓冲区里的指令，并等待所有指令执行完成后再返回。
            TICK("Global");

            if(gui->drawFxaa->Get())
            {
                gui->drawFXAA(gui->s_cam.GetProjectionModelViewMatrix(),
                              gui->s_cam.GetModelViewMatrix(),
                              eFusion->getGlobalModel().model(),
                              eFusion->getConfidenceThreshold(),
                              eFusion->getTick(),
                              eFusion->getTimeDelta(),
                              iclnuim);
            }
            else                       //执行
            {
                eFusion->getGlobalModel().renderPointCloud(gui->s_cam.GetProjectionModelViewMatrix(),
                                                           eFusion->getConfidenceThreshold(),
                                                           gui->drawUnstable->Get(),
                                                           gui->drawNormals->Get(),
                                                           gui->drawColors->Get(),
                                                           gui->drawPoints->Get(),
                                                           gui->drawWindow->Get(),
                                                           gui->drawTimes->Get(),
                                                           eFusion->getTick(),
                                                           eFusion->getTimeDelta());
            }
            glFinish();
            TOCK("Global");
        }

        if(eFusion->getLost())  //Returns whether or not the camera is lost,if relocalisation mode is on
        {
            glColor3f(1, 1, 0);
        }
        else
        {
            glColor3f(1, 0, 1);     //指示相机位置的那个框的颜色设定
        }
        gui->drawFrustum(pose);
        glColor3f(1, 1, 1);

        if(gui->drawFerns->Get())    //不执行
        {
            glColor3f(0, 0, 0);
            for(size_t i = 0; i < eFusion->getFerns().frames.size(); i++)
            {
                if((int)i == eFusion->getFerns().lastClosest)
                    continue;

                gui->drawFrustum(eFusion->getFerns().frames.at(i)->pose);
            }
            glColor3f(1, 1, 1);
        }

        if(gui->drawDefGraph->Get())
        {
            const std::vector<GraphNode*> & graph = eFusion->getLocalDeformation().getGraph();

            for(size_t i = 0; i < graph.size(); i++)
            {
                pangolin::glDrawCross(graph.at(i)->position(0),
                                      graph.at(i)->position(1),
                                      graph.at(i)->position(2),
                                      0.1);

                for(size_t j = 0; j < graph.at(i)->neighbours.size(); j++)
                {
                    pangolin::glDrawLine(graph.at(i)->position(0),
                                         graph.at(i)->position(1),
                                         graph.at(i)->position(2),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(0),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(1),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(2));
                }
            }
        }

        if(eFusion->getFerns().lastClosest != -1)  //不执行
        {
            glColor3f(1, 0, 0);
            //cout<<"111"<<endl;
            gui->drawFrustum(eFusion->getFerns().frames.at(eFusion->getFerns().lastClosest)->pose);
            glColor3f(1, 1, 1);
        }

        const std::vector<PoseMatch> & poseMatches = eFusion->getPoseMatches();

        int maxDiff = 0;
        for(size_t i = 0; i < poseMatches.size(); i++)
        {
            if(poseMatches.at(i).secondId - poseMatches.at(i).firstId > maxDiff)
            {
                maxDiff = poseMatches.at(i).secondId - poseMatches.at(i).firstId;
            }
        }

        for(size_t i = 0; i < poseMatches.size(); i++)
        {
            if(gui->drawDeforms->Get())
            {
                if(poseMatches.at(i).fern)
                {
                    glColor3f(1, 0, 0);
                }
                else
                {
                    glColor3f(0, 1, 0);
                }
                for(size_t j = 0; j < poseMatches.at(i).constraints.size(); j++)
                {
                    pangolin::glDrawLine(poseMatches.at(i).constraints.at(j).sourcePoint(0), poseMatches.at(i).constraints.at(j).sourcePoint(1), poseMatches.at(i).constraints.at(j).sourcePoint(2),
                                         poseMatches.at(i).constraints.at(j).targetPoint(0), poseMatches.at(i).constraints.at(j).targetPoint(1), poseMatches.at(i).constraints.at(j).targetPoint(2));
                }
            }
        }
        glColor3f(1, 1, 1);

        eFusion->normaliseDepth(0.3f, gui->depthCutoff->Get());

        for(std::map<std::string, GPUTexture*>::const_iterator it = eFusion->getTextures().begin(); it != eFusion->getTextures().end(); ++it)
        {
            if(it->second->draw)
            {
                gui->displayImg(it->first, it->second);
            }
        }

        eFusion->getIndexMap().renderDepth(gui->depthCutoff->Get());

        gui->displayImg("ModelImg", eFusion->getIndexMap().imageTex());
        gui->displayImg("Model", eFusion->getIndexMap().drawTex());

        std::stringstream strs;
        strs << eFusion->getGlobalModel().lastCount();
        gui->totalPoints->operator=(strs.str());

        std::stringstream strs2;
        strs2 << eFusion->getLocalDeformation().getGraph().size();
        gui->totalNodes->operator=(strs2.str());

        std::stringstream strs3;
        strs3 << eFusion->getFerns().frames.size();
        gui->totalFerns->operator=(strs3.str());

        std::stringstream strs4;
        strs4 << eFusion->getDeforms();
        gui->totalDefs->operator=(strs4.str());

        std::stringstream strs5;
        strs5 << eFusion->getTick() << "/" << logReader->getNumFrames();
        gui->logProgress->operator=(strs5.str());

        std::stringstream strs6;
        strs6 << eFusion->getFernDeforms();
        gui->totalFernDefs->operator=(strs6.str());

        gui->postCall();

        logReader->flipColors = gui->flipColors->Get();
        eFusion->setRgbOnly(gui->rgbOnly->Get());
        eFusion->setPyramid(gui->pyramid->Get());
        eFusion->setFastOdom(gui->fastOdom->Get());
        eFusion->setConfidenceThreshold(gui->confidenceThreshold->Get());
        eFusion->setDepthCutoff(gui->depthCutoff->Get());
        eFusion->setIcpWeight(gui->icpWeight->Get());
        eFusion->setSo3(gui->so3->Get());
        eFusion->setFrameToFrameRGB(gui->frameToFrameRGB->Get());

        resetButton = pangolin::Pushed(*gui->reset);

        if(gui->autoSettings)                      //执行
        {
            static bool last = gui->autoSettings->Get();
            //std::cout<<"autosetting 111"<<endl;

            if(gui->autoSettings->Get() != last)   //不执行
            {
                last = gui->autoSettings->Get();
                static_cast<LiveLogReader *>(logReader)->setAuto(last);
                //std::cout<<"autosetting 222"<<endl;
            }
        }

        Stopwatch::getInstance().sendAll();

        if(resetButton)
        {
            break;
        }

        if(pangolin::Pushed(*gui->save))
        {
            eFusion->savePly();
        }

        TOCK("GUI");
    }
}
