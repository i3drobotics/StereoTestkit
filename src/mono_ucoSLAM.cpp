#include <iostream>
#include <ucoslam/ucoslam.h>
#include <ucoslam/mapviewer.h>
#include <opencv2/highgui/highgui.hpp>
#include "uvc_camera/deimos_cam.h"

int main(int argc, char **argv)
{
    std::string dev = "/dev/video0";
    std::string cameraYamlFile = "cam.yaml";
    std::string VocabFile = "orb.fbow";
    std::string OutputMapFile = "world.map";

    if (argc > 1)
    {
        if (argc == 5)
        {
            dev = argv[1];
            cameraYamlFile = argv[2];
            VocabFile = argv[3];
            OutputMapFile = argv[4];
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
    std::cout << "Camera Yaml File: " << cameraYamlFile << std::endl;
    std::cout << "Vocab File: " << VocabFile << std::endl;
    std::cout << "Output File: " << OutputMapFile << std::endl;
    std::cout << "=======================" << std::endl;

    uvc_camera::deimosCamera camera("/dev/video0");
    if (camera.isCameraStereo == false)
    {
        std::cerr << "Stereo camera not found" << std::endl;
        return -1;
    }
    else
    {
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

        while (true)
        {
            camera.grabFrame();

            cv::Mat posef2g = SLAM.process(camera.rightImage, cameraParams, frameNumber);
            if (posef2g.empty())
            {
                std::cerr << "Frame " << frameNumber << " pose not found" << std::endl;
            }
            else
                std::cerr << "Frame " << frameNumber << " pose " << posef2g << std::endl;
            //draw a mininimal interface in an opencv window
            //keyPressed = MapViwer.show(map, camera.rightImage, posef2g);
            cv::imshow("right", camera.rightImage);
            cv::imshow("left", camera.leftImage);
            if (keyPressed == 27)
            {
                break;
            }
            frameNumber++;
        }
        //now,  save the map
        map->saveToFile(OutputMapFile);
    }
    return 0;
}