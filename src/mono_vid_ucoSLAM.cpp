#include <iostream>
#include <ucoslam/ucoslam.h>
#include <ucoslam/mapviewer.h>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>

#include <stereoMatcher/matcherOpenCVBlock.h>
#include <stereoMatcher/matcherOpenCVSGBM.h>

#include "PLYData.hpp"

int main(int argc, char **argv)
{
    std::string dev = "/dev/video0";
    std::string cameraYamlFile = "stereo.yaml";
    std::string VocabFile = "orb.fbow";
    std::string OutputFolder = "data/";

    if (argc > 1)
    {
        if (argc == 5)
        {
            dev = argv[1];
            cameraYamlFile = argv[2];
            VocabFile = argv[3];
            OutputFolder = argv[4];
        }
        else
        {
            std::cerr << "Invalid number of arguments" << std::endl;
            return -1;
        }
    }
    else
    {
        std::cout << "Using default paramters" << std::endl;
    }
    std::cout << "=======================" << std::endl;
    std::cout << "Device: " << dev << std::endl;
    std::cout << "Stereo Yaml File: " << cameraYamlFile << std::endl;
    std::cout << "Vocab File: " << VocabFile << std::endl;
    std::cout << "Output Folder: " << OutputFolder << std::endl;
    std::cout << "=======================" << std::endl;

    std::string OutputMapFile = OutputFolder + "world.map";

    cv::VideoCapture cap(dev); 
    // Check if camera opened successfully
    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    } else {
        ucoslam::UcoSlam SLAM;             //The main class
        ucoslam::Params UcoSlamParams;     //processing parameters
        ucoslam::ImageParams cameraParams; //camera parameters
        ucoslam::MapViewer MapViwer;       //Viewer to see the 3D map and the input images
        //creates an empty map
        std::shared_ptr<ucoslam::Map> map = std::make_shared<ucoslam::Map>();
        cameraParams.readFromXMLFile(cameraYamlFile);
        UcoSlamParams.runSequential = true;  //run in sequential mode to avoid skipping frames
        UcoSlamParams.detectMarkers = false; //no markers in this example.

        SLAM.setParams(map, UcoSlamParams, VocabFile); //the last parameter is the path to the vocabulary file of extension .fbow

        unsigned char *right_frame = nullptr;
        unsigned char *left_frame = nullptr;
        char keyPressed = 0;
        int frameNumber = 0;

        AbstractStereoMatcher *matcher = nullptr;
		MatcherOpenCVBlock *block_matcher;
		MatcherOpenCVSGBM *sgbm_matcher;

        int flags = cv::FileStorage::READ;
        cv::Mat CamMatrix, DistCoeff;
        cv::FileStorage fs_stereo(cameraYamlFile, flags);
        if (fs_stereo.isOpened())
        {
            fs_stereo["camera_matrix"] >> CamMatrix;
            CamMatrix.convertTo(CamMatrix, CV_64F);
            fs_stereo["distortion_coefficients"] >> DistCoeff;
            DistCoeff.convertTo(DistCoeff, CV_64F);
        }
        std::cout << "Camera matrix:" << std::endl;
        std::cout << CamMatrix << std::endl;
        std::cout << "Distortion Coefficients:" << std::endl;
        std::cout << DistCoeff << std::endl;

        cv::Mat prev_frame, prev_pose;

        while (true)
        {
            cv::Mat frame;
            // Capture frame-by-frame
            cap >> frame;
        
            // If the frame is empty, break immediately
            if (frame.empty())
                break;

            cv::Mat slam_frame;
            frame.copyTo(slam_frame);

            cv::Mat posef2g = SLAM.process(slam_frame, cameraParams, frameNumber);
            if (posef2g.empty())
            {
                std::cerr << "Frame " << frameNumber << " pose not found" << std::endl;
            }
            else{
                //std::cout << "Frame " << frameNumber << " pose " << posef2g << std::endl;
                cv::Mat posef2g_64;
                posef2g.convertTo(posef2g_64, CV_64F);
                if (!prev_frame.empty()){
                    std::cout << "Trying it out!" << std::endl;

                    //TODO calculate new pose relative to previous
                    //cv::Mat id_4x4 = (cv::Mat_<float>(4,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
                    cv::Mat relPose =  posef2g_64 * prev_pose.inv();
                    std::cout << "Relative pose" << std::endl;
                    std::cout << relPose << std::endl;

                    cv::Mat R1, R2, P1, P2, Q;
                    double x = relPose.at<double>(0,3);
                    double y = relPose.at<double>(1,3);
                    double z = relPose.at<double>(2,3);
                    double r11 = relPose.at<double>(0,0);
                    double r12 = relPose.at<double>(0,1);
                    double r13 = relPose.at<double>(0,2);
                    double r21 = relPose.at<double>(1,0);
                    double r22 = relPose.at<double>(1,1);
                    double r23 = relPose.at<double>(1,2);
                    double r31 = relPose.at<double>(2,0);
                    double r32 = relPose.at<double>(2,1);
                    double r33 = relPose.at<double>(2,2);
                    cv::Mat T = (cv::Mat_<double>(3,1) << x, y, z);
                    cv::Mat R = (cv::Mat_<double>(3,3) << r11,r12,r13,r21,r22,r23,r31,r32,r33);

                    std::cout << "T:" << std::endl;
                    std::cout << T << std::endl;

                    std::cout << "R:" << std::endl;
                    std::cout << R << std::endl;
                                   

                    cv::stereoRectify(CamMatrix, DistCoeff, CamMatrix, DistCoeff, frame.size(), R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0,  frame.size(), 0, 0);
                    
                    cv::Mat left_undist_rect_map_x,left_undist_rect_map_y;
                    cv::Mat right_undist_rect_map_x,right_undist_rect_map_y;
                    initUndistortRectifyMap(CamMatrix, DistCoeff, R1, P1, frame.size(), CV_32FC1, left_undist_rect_map_x, left_undist_rect_map_y);
                    initUndistortRectifyMap(CamMatrix, DistCoeff, R2, P2, frame.size(), CV_32FC1, right_undist_rect_map_x, right_undist_rect_map_y);
                    cv::Mat left_undist_rect,right_undist_rect;
                    cv::remap(prev_frame, left_undist_rect, left_undist_rect_map_x, left_undist_rect_map_y, cv::INTER_CUBIC, cv::BORDER_CONSTANT, 0);
                    cv::remap(frame, right_undist_rect, right_undist_rect_map_x, right_undist_rect_map_y, cv::INTER_CUBIC, cv::BORDER_CONSTANT, 0);
                    
                    cv::imshow("rect left",left_undist_rect);
                    cv::imshow("rect right",right_undist_rect);

                    std::string empty_str = " ";
                    block_matcher = new MatcherOpenCVBlock(empty_str, frame.size());
                    sgbm_matcher = new MatcherOpenCVSGBM(empty_str, frame.size());

                    matcher = sgbm_matcher;

                    matcher->setImages(&left_undist_rect, &right_undist_rect);
                    int exitCode = matcher->match();
                    cv::Mat disparityImage;
                    cv::Mat(frame.size(), CV_32F).copyTo(disparityImage);
                    if (exitCode == 0)
                    {
                        matcher->getDisparity(disparityImage);
                        //std::cout << "Match complete." << std::endl;
                    }
                    else
                    {
                        std::cerr << "Failed to compute stereo match" << std::endl;
                        std::cerr << "Please check parameters are valid." << std::endl;
                    }
                    cv::imshow("disp",disparityImage);
                    cv::Mat disparityImage16, points3D;
                    disparityImage.convertTo(disparityImage16, CV_16S);
                    cv::reprojectImageTo3D(disparityImage16, points3D, Q, false);
                    /*
                    DataExporter exporterT(points3D,                                                    // 3D coordinate matrix to output
                                    left_undist_rect,                                                    // Color matrix to output
                                    OutputFolder + "Points" + std::to_string(frameNumber) + ".ply", // Output file name/path
                                    FileFormat::PLY_BIN_BIGEND);                                    // Output format, can be PLY_ASCII, PLY_BIGEND or PLY_LITEND
                    exporterT.exportToFile();
                    */
                }
                frame.copyTo(prev_frame);
                posef2g_64.copyTo(prev_pose);
            }
            //draw a mininimal interface in an opencv window
            keyPressed = MapViwer.show(map, slam_frame, posef2g);
            if (keyPressed == 27)
            {
                break;
            }
            frameNumber++;
        }
        // When everything done, release the video capture object
        cap.release();
        //now,  save the map
        map->saveToFile(OutputMapFile);
    }
    return 0;
}