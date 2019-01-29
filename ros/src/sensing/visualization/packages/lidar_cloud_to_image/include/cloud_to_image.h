
#ifndef CLOUD_TO_IMAGE_H_
#define CLOUD_TO_IMAGE_H_

#include "projection_params.h"
#include "cloud_projection.h"

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace cloud_to_image
{
class CloudToImage
{
	public:
		explicit CloudToImage();
		~CloudToImage();

		void init(int argc, char* argv[]);
		void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
		void publishImages(const std_msgs::Header& header);

	private:
		//!
		//! \brief getParam Get parameter from node handle
		//! \param nh The nodehandle
		//! \param param_name Key string
		//! \param default_value Default value if not found
		//! \return The parameter value
		//!
		template <typename T>
		T getParam(const ros::NodeHandle& nh, const std::string& param_name, T default_value);

		template <class T>
		static void DoNotFree(T*) {}

		template<class T>
		static std::string timeToStr(T ros_t);

	private:
		CloudProjection* _cloud_proj;

		std::string _cloud_topic;
		std::string _sensor_model;
		std::string _point_type;
		std::string _depth_image_topic;
		std::string _intensity_image_topic;
		std::string _reflectance_image_topic;
		std::string _noise_image_topic;

		bool _has_depth_image;
		bool _has_intensity_image;
		bool _has_reflectance_image;
		bool _has_noise_image;

		float _horizontal_scale;
		float _vertical_scale;

		ros::NodeHandle _nodehandle;
		ros::Subscriber _sub_PointCloud;
		/*image_transport::Publisher*/ ros::Publisher _pub_DepthImage;
		/*image_transport::Publisher*/ ros::Publisher _pub_IntensityImage;
		/*image_transport::Publisher*/ ros::Publisher _pub_ReflectanceImage;
		/*image_transport::Publisher*/ ros::Publisher _pub_NoiseImage;
};
}

#endif // CLOUD_TO_IMAGE_H_