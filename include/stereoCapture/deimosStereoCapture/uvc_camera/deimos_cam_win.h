#include <opencv2/opencv.hpp>
#include <stereoMatcher/matcherOpenCVBlock.h>
#include <stereoMatcher/matcherOpenCVSGBM.h>

/*
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
*/

using namespace std;

#define		M_PI				3.14159265358979323846
#define		HALF_PI				(M_PI / 2)
//#define		DEG2RAD				(M_PI / 180.f)
//#define		RAD2DEG				(180.f / M_PI)

//	1.2.131.652 is the last firmware version of Tara that doesn't support auto exposure.
#define 		MajorVersion_t		1
#define 		MinorVersion1_t	2
#define 		MinorVersion2_t	131
#define 		MinorVersion3_t	652

#define sampleFreq     119.0f                                   // sample frequency in Hz
#define gyroMeasError  0.1                                      // gyroscope measurement error in rad/s
#define betaDef        sqrt(3.0f / 4.0f) * gyroMeasError        // compute beta

void Sleep(unsigned int TimeInMilli);

class deimosCamera {
	public:

		//IMUCONFIG_TypeDef lIMUConfig;
		//IMUDATAINPUT_TypeDef lIMUInput;
		//IMUDATAOUTPUT_TypeDef *lIMUOutput;
		bool isCameraStereo;

		deimosCamera(std::string dev, bool threaded);
		void feedImages();
		void grabFrame();
		int generateDisparity(cv::Mat leftImageRect, cv::Mat rightImageRect);
		int generate3D(cv::Mat disparityImage, cv::Mat Q);
		~deimosCamera();
		int returnValue;
		int width, height, fps;
		cv::Mat leftImage, rightImage, disparityImage, points3D;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloud;
		//int cv_image_type;

	private:
		AbstractStereoMatcher *matcher = nullptr;
		MatcherOpenCVBlock *block_matcher;
		MatcherOpenCVSGBM *sgbm_matcher;

		bool ok;

		int skip_frames, frames_to_skip;
		std::string device, frame;
		std::string frameImageLeft;
		std::string frameImageRight;
		std::string frameCameraInfoLeft;
		std::string frameCameraInfoRight;
		std::string frameIMU;
		int  exposure_value;
		int  brightness_value;
		bool rotate;

		boost::mutex time_mutex_;

		//uvc_cam::Cam *cam;
		//boost::thread image_thread;
		//boost::thread IMU_thread;
		volatile float beta;	// 2 * proportional gain (Kp)
		volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

		double angleX, angleY, angleZ; // Rotational angle for cube [NEW]
		double RwEst[3];

		double squared(double x);
		cv::Mat image2Mat(unsigned char *img);
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr MatToPoinXYZ(cv::Mat leftImageRect, cv::Mat cv_disp, cv::Mat cv_points);
		//double glIMU_Interval;

		//void SetIMUConfigDefaultEnable();
		//void IMU_enable();    
		//int econ_strcmp (const char * str1, const char *str2);
		/*  Returns the interval time for sampling the values of the IMU. */
		//double GetIMUIntervalTime(IMUCONFIG_TypeDef	lIMUConfig);
		//BOOL DisableIMU();
		//BOOL checkFirmware (UINT8 MajorVersion, UINT8 MinorVersion1, UINT16 MinorVersion2, UINT16 MinorVersion3);		//Returns 1 if firmware supports auto exposure, else 0;
		//float invSqrt(float x);
};

