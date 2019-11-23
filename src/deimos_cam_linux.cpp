#include "uvc_cam/uvc_cam.h"
#include "uvc_camera/deimos_cam.h"

namespace uvc_camera
{
deimosCamera::deimosCamera(std::string dev, bool threaded = false)
{
    width = 752;
    height = 480;
    fps = 60;
    //cv_image_type = cv::8UCV_8UC1;
    skip_frames = 0;
    frames_to_skip = 0;
    device = dev;

    std::string empty_str = " ";
    cv::Size image_size(width, height);
    block_matcher = new MatcherOpenCVBlock(empty_str, image_size);
    sgbm_matcher = new MatcherOpenCVSGBM(empty_str, image_size);

    matcher = block_matcher;

    cam = new uvc_cam::Cam(device.c_str(), uvc_cam::Cam::MODE_Y16, width, height, fps);
    if (cam->IsStereo == true)
    {
        isCameraStereo = true;
        cam->showFirmwareVersion();
        unsigned char *in_buffer;
        unsigned char *ex_buffer;
        int intFileLength;
        int extFileLength;

        ok = true;
		/*
        if (threaded)
        {
            //TODO how to read image data without effecting the thread (mutex?)
            image_thread = boost::thread(boost::bind(&deimosCamera::feedImages, this));
        }
		*/
    }
}

void deimosCamera::grabFrame()
{
    uint32_t bytes_used;
    unsigned char *img_frame = nullptr;
    unsigned char *concat_frame = nullptr;
    unsigned char *right_image = nullptr;
    unsigned char *left_image = nullptr;

    int idx = cam->grabStereo(&img_frame, bytes_used, &left_image, &right_image, &concat_frame);

    if (img_frame)
        leftImage = image2Mat(left_image);
    rightImage = image2Mat(right_image);
    cam->release(idx);
}

int deimosCamera::generateDisparity(cv::Mat leftImageRect, cv::Mat rightImageRect)
{
    cv::Mat left_image_rect, right_image_rect;
    leftImageRect.copyTo(left_image_rect);
    rightImageRect.copyTo(right_image_rect);
    cv::Size image_size = cv::Size(left_image_rect.size().width, left_image_rect.size().height);

    // Setup for 32-bit disparity
    cv::Mat(image_size, CV_32F).copyTo(disparityImage);

    matcher->setImages(&left_image_rect, &right_image_rect);

    int exitCode = matcher->match();
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
    return exitCode;
}

int deimosCamera::generate3D(cv::Mat disparityImage, cv::Mat Q)
{
    cv::Mat disparityImage16;
    disparityImage.convertTo(disparityImage16, CV_16S);
    cv::reprojectImageTo3D(disparityImage16, points3D, Q, false);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloudTemp(new pcl::PointCloud<pcl::PointXYZ>);
    return 0;
}

/*
pcl::PointCloud<pcl::PointXYZRGB>::Ptr deimosCamera::MatToPoinXYZ(cv::Mat leftImageRect, cv::Mat cv_disp, cv::Mat cv_points)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloudTemp(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointXYZRGB point;
    uint32_t rgb;
    uchar col;

    for (int i = 0; i < cv_points.rows; i++)
    {

        float *reconst_ptr = cv_points.ptr<float>(i);
        uchar *disp_ptr = cv_disp.ptr<uchar>(i);
        uchar *rgb_ptr = leftImageRect.ptr<uchar>(i);

        for (int j = 0; j < cv_points.cols; j++)
        {

            if (disp_ptr[j] == 0)
                continue;
            if (rgb_ptr[j] == 0)
                continue;

            point.x = reconst_ptr[3 * j];
            point.y = -reconst_ptr[3 * j + 1];
            point.z = -reconst_ptr[3 * j + 2];

            col = rgb_ptr[j];

            rgb = ((int)col) << 16 | ((int)col) << 8 | ((int)col);
            point.rgb = *reinterpret_cast<float *>(&rgb);

            if (abs(point.x) > 1)
                continue;
            if (abs(point.y) > 1)
                continue;
            if (abs(point.z) > 1)
                continue;

            ptCloudTemp->points.push_back(point);
        }
    }

    point.x = 0;
    point.y = 0;
    point.z = 0;

    rgb = ((int)255) << 16 | ((int)255) << 8 | ((int)255);
    point.rgb = *reinterpret_cast<float *>(&rgb);
    ptCloudTemp->points.push_back(point);
    return ptCloudTemp;
}
*/

cv::Mat deimosCamera::image2Mat(unsigned char *img)
{
    cv::Mat cv_image(height, width, CV_8UC1, img, cv::Mat::AUTO_STEP);
    return cv_image;
}

void deimosCamera::feedImages()
{
    unsigned int pair_id = 0;
    while (ok)
    {
        grabFrame();
        if (!leftImage.empty())
        {
            ++pair_id;
        }
    }
}

double deimosCamera::squared(double x)
{
    return x * x;
}

deimosCamera::~deimosCamera()
{
    if (isCameraStereo == true)
    {
        ok = false;
        //image_thread.join();
        //DisableIMU();

        //Freeing the memory
        //free(lIMUOutput);
        //IMU_thread.join();
    }
    if (cam)
        delete cam;
}
} // namespace uvc_camera