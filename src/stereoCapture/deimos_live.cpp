#ifdef _WIN32
    #include "uvc_camera/deimos_cam_win.h"
#endif
#ifdef UNIX
    #include "uvc_camera/deimos_cam_linux.h"
#endif
#include <opencv2/opencv.hpp>

int main (int argc, char **argv)
{ 	    
    std::string dev = "/dev/video0";
    
    if (argc > 1){
        if (argc == 2){
            dev = argv[1];
            std::cout << "Paramters" << std::endl;
            std::cout << "=======================" << std::endl;
            std::cout << "Device: " << dev << std::endl;
            std::cout << "=======================" << std::endl;
        } else {
            std::cerr << "Invalid number of arguments" << std::endl;
            return -1;
        }
    } else {
        std::cout << "Using default paramters" << std::endl;
        std::cout << "=======================" << std::endl;
        std::cout << "Device: " << dev << std::endl;
        std::cout << "=======================" << std::endl;
    }

	uvc_camera::deimosCamera camera("/dev/video0",false);
    if ( camera.isCameraStereo == false )
	{
        return -1;
	}
	else
	{  
		std::cout << "Stereo camera found" << std::endl;
        while(true){
            camera.grabFrame();
            cv::imshow("right",camera.rightImage);
            cv::imshow("left",camera.leftImage);
            if (cv::waitKey(1) == 27) {
                break;
            }
        }
	}
	return 0;
}