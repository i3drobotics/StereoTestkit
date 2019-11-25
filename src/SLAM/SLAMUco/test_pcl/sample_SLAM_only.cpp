#include <iostream>
#include <typeinfo>
#include <ucoslam/ucoslam.h>
#include <ucoslam/mapviewer.h>
#include <ucoslam/stereorectify.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "uvc_camera/deimos_cam.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

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

    uvc_camera::deimosCamera camera("/dev/video0", false);
    if (camera.isCameraStereo == false)
    {
        std::cerr << "Stereo camera not found" << std::endl;
        return -1;
    }
    else
    {
        ucoslam::UcoSlam SLAM;         //The main class
        ucoslam::Params UcoSlamParams; //processing parameters
        ucoslam::StereoRectify StRect; //Stereo camera parameters
        ucoslam::MapViewer MapViwer;   //Viewer to see the 3D map and the input images
        //creates an empty map
        std::shared_ptr<ucoslam::Map> map = std::make_shared<ucoslam::Map>();
        StRect.readFromXMLFile(cameraYamlFile);
        UcoSlamParams.runSequential = true;  //run in sequential mode to avoid skipping frames
        UcoSlamParams.detectMarkers = false; //no markers in this example.

        SLAM.setParams(map, UcoSlamParams, VocabFile); //the last parameter is the path to the vocabulary file of extension .fbow
        SLAM.setDebugLevel(10);
        unsigned char *right_frame = nullptr;
        unsigned char *left_frame = nullptr;
        char keyPressed = 0;
        int frameNumber = 0;
        int exitCode = 0;

        int flags = cv::FileStorage::READ;
        cv::Mat Q;
        cv::FileStorage fs_stereo(cameraYamlFile, flags);
        if (fs_stereo.isOpened())
        {
            fs_stereo["Q"] >> Q;
            Q.convertTo(Q, CV_32F);
        }
        std::cout << Q << std::endl;

        bool random_poses = true;

        while (true)
        {
            camera.grabFrame();

            StRect.rectify(camera.leftImage, camera.rightImage);

            exitCode = camera.generateDisparity(StRect.getLeft(), StRect.getRight());
            if (exitCode == 0)
            {
                camera.generate3D(camera.disparityImage, Q);
                
                pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloudTemp(new pcl::PointCloud<pcl::PointXYZ>);

                cv::imshow("disp", camera.disparityImage);

                cv::Mat posef2g = SLAM.processStereo(StRect.getLeft(), StRect.getRight(), StRect.getImageParams(), frameNumber);
                if (posef2g.empty())
                {
                    std::cerr << "Frame " << frameNumber << " pose not found" << std::endl;
                }
                else
                {
                    std::cout << "Frame " << frameNumber << " pose " << posef2g << std::endl;
                }
                //draw a mininimal interface in an opencv window
                keyPressed = MapViwer.show(map, StRect.getLeft(), posef2g);

                cv::waitKey(1);
                if (keyPressed == 27)
                {
                    break;
                }
                frameNumber++;
            }
            else
            {
                std::cerr << "Failed to generate disparity" << std::endl;
                return exitCode;
            }
        }
        //now, save the map
        map->saveToFile(OutputMapFile);
    }
    return 0;
}