#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <opencv2/ximgproc.hpp>

#include <boost/filesystem.hpp>

#include <stereoMatcher/matcherOpenCVBlock.h>
#include <stereoMatcher/matcherOpenCVSGBM.h>

//#define ENABLE_I3DR_ALG ON //TODO: REMOVE THIS

#ifdef ENABLE_I3DR_ALG
#include <stereoMatcher/matcherJRSGM.h>
#endif

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include <string>

using namespace cv;

typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudRGBNormal;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

AbstractStereoMatcher *matcher = nullptr;
MatcherOpenCVBlock *block_matcher;
MatcherOpenCVSGBM *sgbm_matcher;
#ifdef ENABLE_I3DR_ALG
MatcherJRSGM *jrsgm_matcher;
#endif

float _depth_max = 10;

bool isFirstImagesRecevied = false;

int CV_StereoBM = 0;
int CV_StereoSGBM = 1;
int JR_StereoSGM = 2;
int _stereo_algorithm = CV_StereoBM;

int _min_disparity = 9;
int _disparity_range = 64;
int _correlation_window_size = 15;
int _uniqueness_ratio = 15;
int _texture_threshold = 10;
int _speckle_size = 100;
int _speckle_range = 4;
int _disp12MaxDiff = 0;
float _p1 = 200;
float _p2 = 400;
bool _interp = false;
int _preFilterCap = 31;
int _preFilterSize = 9;

std::string _jr_config_file = "";

cv::Mat _Kl, _Dl, _Rl, _Pl;
cv::Mat _Kr, _Dr, _Rr, _Pr;

cv::Mat _stereo_left, _stereo_right, _stereo_left_rect, _stereo_right_rect, _stereo_disparity;
PointCloudRGB::Ptr _stereo_point_cloud_RGB;

bool isInitParamConfig = true;
int save_index = 0;

//Convert disparity image from opencv Mat to PCL Point Cloud XYZRGB
PointCloudRGB::Ptr Mat2PCL(cv::Mat image, cv::Mat coords, PointCloudRGBNormal::Ptr normals)
{

  PointCloudRGB::Ptr ptCloudTemp(new PointCloudRGB);
  PointCloudRGBNormal::Ptr ptCloudNormals(new PointCloudRGBNormal);

  pcl::PointXYZRGB point;
  pcl::PointXYZRGBNormal pNormal;
  uint32_t rgb = 0;
  uchar col = 0;

  point.x = 0;
  point.y = 0;
  point.z = 0;

  pNormal.x = 0;
  pNormal.y = 0;
  pNormal.z = 0;

  pNormal.normal_x = 0;
  pNormal.normal_y = 0;
  pNormal.normal_z = 0;

  rgb = ((int)255) << 16 | ((int)255) << 8 | ((int)255);
  point.rgb = *reinterpret_cast<float *>(&rgb);

  pNormal.rgb = point.rgb;

  ptCloudTemp->points.push_back(point);
  ptCloudNormals->points.push_back(pNormal);

  for (int i = 1; i < coords.rows - 1; i++)
  {
    float *reconst_ptr = coords.ptr<float>(i);
    uchar *rgb_ptr = image.ptr<uchar>(i);

    if (!rgb_ptr || !reconst_ptr)
      return (ptCloudTemp);

    for (int j = 1; j < coords.cols - 1; j++)
    {
      if (rgb_ptr[j] == 0)
        continue;

      point.x = reconst_ptr[3 * j];
      point.y = reconst_ptr[3 * j + 1];
      point.z = reconst_ptr[3 * j + 2];

      if (abs(point.x) > 50)
        continue;
      if (abs(point.y) > 50)
        continue;
      if (abs(point.z) > 50 || point.z < 0)
        continue;
      col = rgb_ptr[j];

      rgb = ((int)col) << 16 | ((int)col) << 8 | ((int)col);
      point.rgb = *reinterpret_cast<float *>(&rgb);

      //normals
      float dzdx = (coords.at<float>(i + 1, j) - coords.at<float>(i - 1, j)) / 2.0;
      float dzdy = (coords.at<float>(i, j + 1) - coords.at<float>(i, j - 1)) / 2.0;

      cv::Vec3f d(-dzdx, -dzdy, 1.0f);

      cv::Vec3f n = normalize(d);

      pNormal.x = point.x;
      pNormal.y = point.y;
      pNormal.z = point.z;

      pNormal.rgb = point.rgb;

      pNormal.normal_x = n[0];
      pNormal.normal_y = n[1];
      pNormal.normal_z = n[2];

      ptCloudNormals->points.push_back(pNormal);

      ptCloudTemp->points.push_back(point);
    }
  }
  pcl::copyPointCloud(*ptCloudNormals, *normals);
  return (ptCloudTemp);
}

void updateMatcher(){
  std::cout << "Updating matcher parameters..." << std::endl;

  if (_stereo_algorithm == CV_StereoBM || _stereo_algorithm == CV_StereoSGBM)
  {
    matcher->setDisparityRange(_disparity_range);
    matcher->setWindowSize(_correlation_window_size);
    matcher->setInterpolation(_interp);
    //Functions unique to OpenCV Stereo BM & SGBM
    matcher->setMinDisparity(_min_disparity);
    matcher->setUniquenessRatio(_uniqueness_ratio);
    matcher->setSpeckleFilterRange(_speckle_range);
    matcher->setSpeckleFilterWindow(_speckle_size);
    matcher->setPreFilterCap(_preFilterCap);
  }
  if (_stereo_algorithm == CV_StereoBM)
  {
    //Functions unique to OpenCV Stereo BM
    matcher->setTextureThreshold(_texture_threshold);
    matcher->setPreFilterSize(_preFilterSize);
  }
  else if (_stereo_algorithm == CV_StereoSGBM)
  {
    //Functions unique to OpenCV Stereo SGBM
    matcher->setP1(_p1);
    matcher->setP2(_p2);
  }
  else if (_stereo_algorithm == JR_StereoSGM)
  {
    matcher->setDisparityRange(_disparity_range);
    matcher->setWindowSize(_correlation_window_size);
    matcher->setInterpolation(_interp);
    matcher->setMinDisparity(_min_disparity);
    //Functions unique to JR Stereo SGM
    matcher->setP1(_p1);
    matcher->setP2(_p2);
    //TODO setup global parameters for occluision
    //bool occlusion = false;
    //matcher->setOcclusionDetection(occlusion);
  }
  std::cout << "Matcher parameters updated." << std::endl;
}

void init_matcher(cv::Size image_size){
  std::cout << "Initalisating matching on first use..." << std::endl;
  std::string empty_str = " ";

  block_matcher = new MatcherOpenCVBlock(empty_str,image_size);
  sgbm_matcher = new MatcherOpenCVSGBM(empty_str,image_size);
#ifdef ENABLE_I3DR_ALG
  jrsgm_matcher = new MatcherJRSGM(_jr_config_file,image_size);
#endif

  if (_stereo_algorithm == CV_StereoBM)
  {
    matcher = block_matcher;
  }
  else if (_stereo_algorithm == CV_StereoSGBM)
  {
    matcher = sgbm_matcher;
  }
  else if (_stereo_algorithm == JR_StereoSGM)
  {
#ifdef ENABLE_I3DR_ALG
    matcher = jrsgm_matcher;
#else
    matcher = block_matcher;
    _stereo_algorithm = CV_StereoBM;
    std::cerr<< "Not built to use I3DR algorithm. Resetting to block matcher." << std::endl;
#endif
  }
  std::cout << "Matcher initalised." << std::endl;

  updateMatcher();
}

//Calculate disparity using left and right images
Mat stereo_match(Mat left_image, Mat right_image)
{
  cv::Mat disp;
  cv::Size image_size = cv::Size(left_image.size().width, left_image.size().height);

  // Setup for 16-bit disparity
  cv::Mat(image_size, CV_32F).copyTo(disp);

  if (!isFirstImagesRecevied){
    init_matcher(image_size);
    isFirstImagesRecevied = true;
  }

  //std::cout << "Matching..." << std::endl;

  matcher->setImages(&left_image, &right_image);

  int exitCode = matcher->match();
  if (exitCode == 0){
    matcher->getDisparity(disp);
    //std::cout << "Match complete." << std::endl;
  } else {
    std::cerr << "Failed to compute stereo match" << std::endl;
    std::cerr << "Please check parameters are valid." << std::endl;
  }

  return disp;
}

cv::Mat_<uint8_t> rectify(cv::Mat image, cv::Mat K, cv::Mat D, cv::Mat R, cv::Mat P)
{
  cv::Size resol = cv::Size(image.size().width, image.size().height);

  cv::Mat full_map1, full_map2;

  cv::initUndistortRectifyMap(K, D, R, P, resol,
                              CV_32FC1, full_map1, full_map2);

  cv::Mat_<uint8_t> image_rect;
  cv::remap(image, image_rect, full_map1, full_map2, cv::INTER_CUBIC);

  return (image_rect);
}

int processDisparity(const cv::Mat &left_rect, const cv::Mat &right_rect,
                      cv::Mat K, cv::Mat D, cv::Mat R, cv::Mat P,
                      cv::Mat &disparity)
{
}

int main(int argc, char **argv)
{
  int stereo_algorithm, min_disparity, disparity_range, correlation_window_size, uniqueness_ratio, texture_threshold, speckle_size, speckle_range, disp12MaxDiff, preFilterCap, preFilterSize;

  float p1, p2;
  bool interp;
  std::string frame_id, left_camera_calibration_url, right_camera_calibration_url, jr_config_file;
  float depth_max;

  std::string ns = ros::this_node::getNamespace();

  //Get parameters
  if (p_nh.getParam("stereo_algorithm", stereo_algorithm))
  {
    _stereo_algorithm = stereo_algorithm;
    ROS_INFO("stereo_algorithm: %d", _stereo_algorithm);
  }
  if (p_nh.getParam("min_disparity", min_disparity))
  {
    _min_disparity = min_disparity;
    ROS_INFO("min_disparity: %d", _min_disparity);
  }
  if (p_nh.getParam("disparity_range", disparity_range))
  {
    _disparity_range = disparity_range;
    ROS_INFO("disparity_range: %d", _disparity_range);
  }
  if (p_nh.getParam("correlation_window_size", correlation_window_size))
  {
    _correlation_window_size = correlation_window_size;
    ROS_INFO("correlation_window_size: %d", _correlation_window_size);
  }
  if (p_nh.getParam("uniqueness_ratio", uniqueness_ratio))
  {
    _uniqueness_ratio = uniqueness_ratio;
    ROS_INFO("uniqueness_ratio: %d", _uniqueness_ratio);
  }
  if (p_nh.getParam("texture_threshold", texture_threshold))
  {
    _texture_threshold = texture_threshold;
    ROS_INFO("texture_threshold: %d", _texture_threshold);
  }
  if (p_nh.getParam("speckle_size", speckle_size))
  {
    _speckle_size = speckle_size;
    ROS_INFO("speckle_size: %d", _speckle_size);
  }
  if (p_nh.getParam("speckle_range", speckle_range))
  {
    _speckle_range = speckle_range;
    ROS_INFO("speckle_range: %d", _speckle_range);
  }
  if (p_nh.getParam("disp12MaxDiff", disp12MaxDiff))
  {
    _disp12MaxDiff = disp12MaxDiff;
    ROS_INFO("disp12MaxDiff: %d", _disp12MaxDiff);
  }
  if (p_nh.getParam("p1", p1))
  {
    _p1 = p1;
    ROS_INFO("p1: %f", _p1);
  }
  if (p_nh.getParam("p2", p2))
  {
    _p2 = p2;
    ROS_INFO("p2: %f", _p2);
  }
  if (p_nh.getParam("pre_filter_cap", preFilterCap))
  {
    _preFilterCap = preFilterCap;
    ROS_INFO("pre_filter_cap %d", _preFilterCap);
  }
  if (p_nh.getParam("pre_filter_size", preFilterSize))
  {
    _preFilterSize = preFilterSize;
    ROS_INFO("pre_filter_size %d", _preFilterSize);
  }
  if (p_nh.getParam("frame_id", frame_id))
  {
    _frame_id = frame_id;
    ROS_INFO("frame_id: %s", _frame_id.c_str());
  } else {
    _frame_id = ns+"_depth_optical_frame";
  }
  if (p_nh.getParam("jr_config_file", jr_config_file))
  {
    _jr_config_file = jr_config_file;
    ROS_INFO("jr_config_file: %s", _jr_config_file.c_str());
  }
  if (p_nh.getParam("interp", interp))
  {
    _interp = interp;
    ROS_INFO("interp: %s", interp ? "true" : "false");
  }
  if (p_nh.getParam("depth_max", depth_max))
  {
    _depth_max = depth_max;
    ROS_INFO("depth_max: %f", depth_max);
  }

  // Dynamic parameters
  dynamic_reconfigure::Server<i3dr_stereo_camera::i3DR_DisparityConfig> server;
  dynamic_reconfigure::Server<i3dr_stereo_camera::i3DR_DisparityConfig>::CallbackType f;
  f = boost::bind(&parameterCallback, _1, _2);
  server.setCallback(f);

  // Publishers creation
  _disparity_pub = nh.advertise<stereo_msgs::DisparityImage>(ns + "/disparity", 1, true);
  _rect_l_pub = nh.advertise<sensor_msgs::Image>(ns + "/left/image_rect", 1, true);
  _rect_r_pub = nh.advertise<sensor_msgs::Image>(ns + "/right/image_rect", 1, true);
  _point_cloud_pub = nh.advertise<PointCloudRGB>(ns + "/points2", 1);
  _point_cloud_normal_pub = nh.advertise<PointCloudRGBNormal>(ns + "/points2_normal", 1);

  // Start services
  ros::ServiceServer srv_save_stereo = nh.advertiseService("save_stereo", save_stereo);

  // Subscribers creation.
  message_filters::Subscriber<sensor_msgs::Image> sub_image_l(nh, ns + "/left/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_image_r(nh, ns + "/right/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_l(nh, ns + "/left/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_r(nh, ns + "/right/camera_info", 1);

  // Message filter creation.
  message_filters::Synchronizer<policy_t> sync(policy_t(10), sub_image_l, sub_image_r, sub_camera_info_l, sub_camera_info_r);
  sync.registerCallback(boost::bind(&imageCb, _1, _2, _3, _4));

  ros::spin();
}