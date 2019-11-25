#include <iostream>
#include <typeinfo>
#include <ucoslam/ucoslam.h>
#include <ucoslam/mapviewer.h>
#include <ucoslam/stereorectify.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "uvc_camera/deimos_cam.h"
#include "PLYData.hpp"

cv::Mat createOne(std::vector<cv::Mat> & images, int cols, int min_gap_size)
{
    // let's first find out the maximum dimensions
    int max_width = 0;
    int max_height = 0;
    for ( int i = 0; i < images.size(); i++) {
        // check if type is correct 
        // you could actually remove that check and convert the image 
        // in question to a specific type
        if ( i > 0 && images[i].type() != images[i-1].type() ) {
            std::cerr << "WARNING:createOne failed, different types of images";
            return cv::Mat();
        }
        max_height = std::max(max_height, images[i].rows);
        max_width = std::max(max_width, images[i].cols);
    }
    // number of images in y direction
    int rows = std::ceil(images.size() / (float) cols);

    // create our result-matrix
    cv::Mat result = cv::Mat::zeros(rows*max_height + (rows-1)*min_gap_size,
                                    cols*max_width + (cols-1)*min_gap_size, images[0].type());
    size_t i = 0;
    int current_height = 0;
    int current_width = 0;
    for ( int y = 0; y < rows; y++ ) {
        for ( int x = 0; x < cols; x++ ) {
            if ( i >= images.size() ) // shouldn't happen, but let's be safe
                return result;
            // get the ROI in our result-image
            cv::Mat to(result,
                       cv::Range(current_height, current_height + images[i].rows),
                       cv::Range(current_width, current_width + images[i].cols));
            // copy the current image to the ROI
            images[i++].copyTo(to);
            current_width += max_width + min_gap_size;
        }
        // next line - reset width and update height
        current_width = 0;
        current_height += max_height + min_gap_size;
    }
    return result;
}

cv::Mat removeRow(cv::Mat matIn,int row){
    // Removing a row
    cv::Size size = matIn.size();
    cv::Mat matOut (cv::Size(size.width,size.height - 1),matIn.type());

    std::cout << "copying above of row" << std::endl;

    if ( row > 0 ) // Copy everything above that one row.
    {
        cv::Rect rect( 0, 0, size.width, row );
        matIn( rect ).copyTo( matOut( rect ) );
    }

    std::cout << "copying below of row" << std::endl;

    if ( row < size.height - 1 ) // Copy everything below that one row.
    {
        cv::Rect rect1( 0, row + 1, size.width, size.height - row - 1 );
        cv::Rect rect2( 0, row, size.width, size.height - row - 1 );
        matIn( rect1 ).copyTo( matOut( rect2 ) );
    }
    return matOut;
}

cv::Mat removeCol(cv::Mat matIn,int col){
    // Removing a column
    
    cv::Size size = matIn.size();
    cv::Mat matOut (cv::Size(size.width - 1,size.height),matIn.type());

    std::cout << "copying left of column" << std::endl;

    // Removing a column
    if ( col > 0 ) // Copy everything left of that one column.
    {
        cv::Rect rect( 0, 0, col, size.height );
        matIn( rect ).copyTo( matOut( rect ) );
    }

    std::cout << "copying right of column" << std::endl;

    if ( col < size.width - 1 ) // Copy everything right of that one column.
    {
        cv::Rect rect1( col + 1, 0, size.width - col - 1, size.height );
        cv::Rect rect2( col,     0, size.width - col - 1, size.height );
        matIn( rect1 ).copyTo( matOut( rect2 ) );
    }
    return matOut;
}

cv::Mat unorganiseColorMat(cv::Mat organised_mat){
    cv::Mat unorganised_mat = cv::Mat::zeros(cv::Size(1, organised_mat.rows*organised_mat.cols), CV_8UC1);
    for (int i = 0; i < organised_mat.rows; i++)
    {
        float *reconst_ptr = organised_mat.ptr<float>(i);
        for (int j = 0; j < organised_mat.cols; j++)
        {
            //uchar b = reconst_ptr[3 * j];
            //float g = reconst_ptr[3 * j + 1];
            //float r = reconst_ptr[3 * j + 2];
            uchar r,g,b;
            r = g = b = organised_mat.at<uchar>(i,j);
            unorganised_mat.at<uchar>(i*j,0) = r;
            //unorganised_mat.at<float>(i*j,1) = g;
            //unorganised_mat.at<float>(i*j,2) = r;
        }
    }
    std::cout << "organised colour : " << organised_mat.size << std::endl;
    std::cout << "unorganised colour: " << unorganised_mat.size << std::endl;
    return unorganised_mat;
}

cv::Mat transposeMat2(cv::Mat mat1, cv::Mat mat2){
    cv::Mat newMat;
    mat1.copyTo(newMat);
    cv::Mat betterMat = cv::Mat::zeros(cv::Size(4, mat1.rows*mat1.cols), CV_32F);
    for (int i = 0; i < mat1.rows; i++)
    {
        float *reconst_ptr = mat1.ptr<float>(i);
        for (int j = 0; j < mat1.cols; j++)
        {
            float x = reconst_ptr[3 * j];
            float y = reconst_ptr[3 * j + 1];
            float z = reconst_ptr[3 * j + 2];
            cv::Mat id = (cv::Mat_<float>(4,4) << 1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1);
            cv::Mat t_point = id*mat2;
            float n[3] = {t_point.at<float>(0,3),t_point.at<float>(1,3),t_point.at<float>(2,3)};
            cv::Vec3f & xyz = newMat.at<cv::Vec3f>(i,j);
            xyz[0] = n[0];
            xyz[1] = n[1];
            xyz[2] = n[2];
            betterMat.at<float>(i*j,0) = x;
            betterMat.at<float>(i*j,1) = y;
            betterMat.at<float>(i*j,2) = z;
            betterMat.at<float>(i*j,3) = 1;
        }
    }
    std::cout << "unorganised: " << betterMat.size << std::endl;
    std::cout << "pose: " << mat2.size << std::endl;
    cv::Mat tmp_mat = betterMat*mat2;
    std::cout << "transformed: " << tmp_mat.size << std::endl;
    cv::Mat t_mat = removeCol(tmp_mat,3);
    std::cout << "resized: " << t_mat.size << std::endl;
    return t_mat;
}

float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

int saveMapData(std::string OutputFolder, std::vector<cv::Mat> point_array, std::vector<cv::Mat> colour_array, std::vector<cv::Mat> slam_poses)
{
    if (!(point_array.empty() && slam_poses.empty())){
        //check both vectors are the same size
        if (point_array.size() == slam_poses.size())
        {
            int flags = cv::FileStorage::WRITE;
            int count = 0;
            std::vector<cv::Mat> t_point_array;
            std::vector<cv::Mat> t_colour_array;
            for (std::vector<cv::Mat>::iterator it = point_array.begin(); it != point_array.end(); ++it)
            {
                std::cout << "Processing points... " << count << std::endl;
                cv::Mat points = *it;
                cv::Mat colors = colour_array[count];
                cv::Mat pose = slam_poses[count];

                cv::Mat TPoints = transposeMat2(points,pose);
                cv::Mat TColors = unorganiseColorMat(colors);
                t_point_array.push_back(TPoints);
                t_colour_array.push_back(TColors);
                count++;
            }
            
            std::cout << "building map..." << std::endl;
            cv::Mat TPointsFull = createOne(t_point_array,1,0);
            cv::Mat ColorsFull = createOne(t_colour_array,1,0);
            std::cout << "saving map PLY.." << std::endl;
            std::cout << "size: " << TPointsFull.size << std::endl;
            DataExporter exporterT(TPointsFull,                                                    // 3D coordinate matrix to output
                                    ColorsFull,                                                    // Color matrix to output
                                    OutputFolder + "Points_T" + ".ply", // Output file name/path
                                    FileFormat::PLY_BIN_BIGEND);                                    // Output format, can be PLY_ASCII, PLY_BIGEND or PLY_LITEND
            exporterT.exportToFile();
            
        } else {
             std::cerr << "both points and poses must be the same size vector" << std::endl;
            return -1;
        }
    }
    else
    {
        std::cerr << "points or poses are empty" << std::endl;
        return -1;
    }
    return 0;
}

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

    uvc_camera::deimosCamera camera("/dev/video0");
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

        std::vector<cv::Mat> point_array;
        std::vector<cv::Mat> colour_array;
        std::vector<cv::Mat> slam_poses;

        bool random_poses = true;

        while (true)
        {
            camera.grabFrame();

            StRect.rectify(camera.leftImage, camera.rightImage);

            exitCode = camera.generateDisparity(StRect.getLeft(), StRect.getRight());
            if (exitCode == 0)
            {

                camera.generate3D(camera.disparityImage, Q);

                cv::imshow("disp", camera.disparityImage);
                std::cout << camera.points3D.size() << std::endl;

                cv::Mat posef2g = SLAM.processStereo(StRect.getLeft(), StRect.getRight(), StRect.getImageParams(), frameNumber);
                if (posef2g.empty())
                {
                    std::cerr << "Frame " << frameNumber << " pose not found" << std::endl;
                    point_array.push_back(camera.points3D);
                    colour_array.push_back(StRect.getLeft());

                    if (random_poses){
                        int rand_x = RandomFloat(-10,10);
                        int rand_y = RandomFloat(-10,10);
                        int rand_z = RandomFloat(-10,10);
                        cv::Mat id = (cv::Mat_<float>(4,4) << RandomFloat(-1,1), RandomFloat(-1,1), RandomFloat(-1,1), rand_x, RandomFloat(-1,1), RandomFloat(-1,1), RandomFloat(-1,1), rand_y, RandomFloat(-1,1), RandomFloat(-1,1), RandomFloat(-1,1), rand_z, 0, 0, 0, 1);
                        slam_poses.push_back(id);
                        std::cout << "Simulated pose" << id << std::endl;
                    }
                }
                else
                {
                    std::cout << "Frame " << frameNumber << " pose " << posef2g << std::endl;
                    point_array.push_back(camera.points3D);
                    colour_array.push_back(StRect.getLeft());
                    slam_poses.push_back(posef2g);
                    
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
        std::cout << "Processing map data... " << std::endl;
        exitCode = saveMapData(OutputFolder, point_array, colour_array, slam_poses);
        map->saveToFile(OutputMapFile);
        std::cout << "Map files saved." << std::endl;
    }
    return 0;
}