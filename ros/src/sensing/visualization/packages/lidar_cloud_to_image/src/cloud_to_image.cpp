#include "cloud_to_image.h"
#include "projection_params.h"
#include "cloud_projection.h"
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/algorithm/string.hpp>

namespace cloud_to_image
{

CloudToImage::CloudToImage():
	_has_depth_image(false),
	_has_intensity_image(false),
	_has_reflectance_image(false),
	_has_noise_image(false)
{	
}

CloudToImage::~CloudToImage() 
{
	if (_cloud_proj) {
		delete _cloud_proj;
	}
	_nodehandle.deleteParam("/cloud2image/cloud_topic");
	_nodehandle.deleteParam("/cloud2image/proj_params");
	_nodehandle.deleteParam("/cloud2image/sensor_model");
	_nodehandle.deleteParam("/cloud2image/point_type");

	_nodehandle.deleteParam("/cloud2image/depth_image_topic");
	_nodehandle.deleteParam("/cloud2image/intensity_image_topic");
	_nodehandle.deleteParam("/cloud2image/reflectance_image_topic");
	_nodehandle.deleteParam("/cloud2image/noise_image_topic");
}

template <typename T>
T CloudToImage::getParam(const ros::NodeHandle& nh, const std::string& param_name, T default_value)
{
	T value;
	if (nh.hasParam(param_name))
  	{
		nh.getParam(param_name, value);
  	}
  	else
  	{
		ROS_WARN_STREAM("Parameter '" << param_name << "' not found, defaults to '" << default_value << "'");
		value = default_value;
  	}
  	return value;
}

void CloudToImage::init(int argc, char* argv[])
{
	(void)argc;
	(void)argv;
	std::string config_filename;
	_nodehandle = ros::NodeHandle("~");

	//read all parameters
	config_filename = getParam(_nodehandle, "/cloud2image/proj_params", std::string(""));

	_cloud_topic = getParam(_nodehandle, "/cloud2image/cloud_topic", std::string("/points_raw"));
	_sensor_model = getParam(_nodehandle, "/cloud2image/sensor_model", std::string("VLP-16"));
	_point_type = getParam(_nodehandle, "/cloud2image/point_type", std::string("XYZI"));

	_depth_image_topic = getParam(_nodehandle, "/cloud2image/depth_image_topic", std::string("/c2i_depth_image"));
	_intensity_image_topic = getParam(_nodehandle, "/cloud2image/intensity_image_topic", std::string("/c2i_intensity_image"));
	_reflectance_image_topic = getParam(_nodehandle, "/cloud2image/reflectance_image_topic", std::string("/c2i_reflectance_image"));
	_noise_image_topic = getParam(_nodehandle, "/cloud2image/noise_image_topic", std::string("/c2i_noise_image"));

	_horizontal_scale = getParam(_nodehandle, "/cloud2image/h_scale", 1.0);
	_vertical_scale = getParam(_nodehandle, "/cloud2image/v_scale", 1.0);


	if (boost::iequals(_point_type, "XYZ")) {  //no intensity (ex. Velodyne, Ouster)
		_has_depth_image = true;
	} else if (boost::iequals(_point_type, "XYZI")) { //with intensity (ex. Velodyne, Ouster)
		_has_depth_image = true;
		_has_intensity_image = true;
	} else if (boost::iequals(_point_type, "XYZIR")) { //with intensity (ex. Velodyne, Ouster)
		_has_depth_image = true;
		_has_intensity_image = true;
	} else if (boost::iequals(_point_type, "XYZIF")) { //with intensity and reflectance (ex. Ouster)
		_has_depth_image = true;
		_has_intensity_image = true;
		_has_reflectance_image = true;
	} else if (boost::iequals(_point_type, "XYZIFN")) { //with intensity and reflectance and noise (ex. Ouster)
		_has_depth_image = true;
		_has_intensity_image = true;
		_has_reflectance_image = true;
		_has_noise_image = true;
	} else {
		throw std::runtime_error("point type \"" + _point_type + "\" not supported");
	}

	//verifies the configuration file
	std::ifstream config(config_filename.c_str());
	if (!config.good()) {
		throw std::runtime_error("Projection parameters file \"" + config_filename + "\" does not exist or invalid path");	
	}

	//loads projection parameters from config file
	ProjectionParams proj_params;
	proj_params.loadFromFile(config_filename);
	//verifies the sensor model
	if (!proj_params.sensorExists(_sensor_model)) {
		throw std::runtime_error("Sensor model \"" + _sensor_model + "\" does not exist in configuration file");	
	}
	//creates the cloud projection for the sensor model
	_cloud_proj = new CloudProjection(*proj_params[_sensor_model]);

	//Mossman corrections seem to help on Velodyne
	if (boost::iequals(_sensor_model, "HDL-64") || boost::iequals(_sensor_model, "HDL-32") || boost::iequals(_sensor_model, "VLP-16")) {
		_cloud_proj->loadMossmanCorrections();
	}

	//create the subscribers and publishers
	_sub_PointCloud = _nodehandle.subscribe(_cloud_topic, 10, &CloudToImage::pointCloudCallback, this);
	
	//publishers
	// image_transport::ImageTransport img_transp(_nodehandle);
	// _pub_DepthImage = img_transp.advertise(_depth_image_topic, 10);
	// _pub_IntensityImage = img_transp.advertise(_intensity_image_topic, 10);
	// _pub_ReflectanceImage = img_transp.advertise(_reflectance_image_topic, 10);
	// _pub_NoiseImage = img_transp.advertise(_noise_image_topic, 10);
	_pub_DepthImage = _nodehandle.advertise<sensor_msgs::Image>(_depth_image_topic, 10);
	_pub_IntensityImage = _nodehandle.advertise<sensor_msgs::Image>(_intensity_image_topic, 10);
	_pub_ReflectanceImage = _nodehandle.advertise<sensor_msgs::Image>(_reflectance_image_topic, 10);
	_pub_NoiseImage = _nodehandle.advertise<sensor_msgs::Image>(_noise_image_topic, 10);
}

void CloudToImage::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	//clear any previous projection data
	_cloud_proj->clearData();

	if (boost::iequals(_point_type, "XYZ")) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*input, *cloud_ptr);
    	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZ> >);
		_cloud_proj->initFromPoints(c_cloud_ptr);
	} else if (boost::iequals(_point_type, "XYZIR")) { 
    	pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZIR>);	
		pcl::fromROSMsg(*input, *cloud_ptr);
    	const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZIR> >);
		_cloud_proj->initFromPoints(c_cloud_ptr);
	} else if (boost::iequals(_point_type, "XYZI")) { 
    	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);	
		pcl::fromROSMsg(*input, *cloud_ptr);
    	const pcl::PointCloud<pcl::PointXYZI>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZI> >);
		_cloud_proj->initFromPoints(c_cloud_ptr);
	} else if (boost::iequals(_point_type, "XYZIF")) { 
    	pcl::PointCloud<pcl::PointXYZIF>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZIF>);	
		pcl::fromROSMsg(*input, *cloud_ptr);
    	const pcl::PointCloud<pcl::PointXYZIF>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZIF> >);
		_cloud_proj->initFromPoints(c_cloud_ptr);
	} else if (boost::iequals(_point_type, "XYZIFN")) { 
    	pcl::PointCloud<pcl::PointXYZIFN>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZIFN>);	
		pcl::fromROSMsg(*input, *cloud_ptr);
    	const pcl::PointCloud<pcl::PointXYZIFN>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZIFN> >);
		_cloud_proj->initFromPoints(c_cloud_ptr);
	}

	publishImages(input->header);
}


void CloudToImage::publishImages(const std_msgs::Header& header)
{
	if (_has_depth_image && _pub_DepthImage.getNumSubscribers() > 0) {
		//depth image is float, normalize it for 16 bits
		cv::Mat mono16_img = cv::Mat(_cloud_proj->depth_image().size(), CV_16UC1);
		cv::normalize(_cloud_proj->depth_image(), mono16_img, 0.0, 65535.0, cv::NORM_MINMAX, CV_16UC1);
		cv::resize(mono16_img, mono16_img, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
		sensor_msgs::ImagePtr depth_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, mono16_img).toImageMsg();
		_pub_DepthImage.publish(depth_img);
	}
	if (_has_intensity_image && _pub_IntensityImage.getNumSubscribers() > 0) {
		cv::Mat mono16_img = cv::Mat(_cloud_proj->intensity_image().size(), CV_16UC1);
		cv::normalize(_cloud_proj->intensity_image(), mono16_img, 0.0, 65535.0, cv::NORM_MINMAX, CV_16UC1);
		cv::resize(mono16_img, mono16_img, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
		sensor_msgs::ImagePtr intensity_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, mono16_img).toImageMsg();
		_pub_IntensityImage.publish(intensity_img);
	}
	if (_has_reflectance_image && _pub_ReflectanceImage.getNumSubscribers() > 0) {
		cv::Mat mono16_img = cv::Mat(_cloud_proj->reflectance_image().size(), CV_16UC1);
		cv::normalize(_cloud_proj->reflectance_image(), mono16_img, 0.0, 65535.0, cv::NORM_MINMAX, CV_16UC1);
		cv::resize(mono16_img, mono16_img, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
		sensor_msgs::ImagePtr reflectance_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, mono16_img).toImageMsg();
		_pub_ReflectanceImage.publish(reflectance_img);
	}
	if (_has_noise_image && _pub_NoiseImage.getNumSubscribers() > 0) {
		cv::Mat mono16_img = cv::Mat(_cloud_proj->noise_image().size(), CV_16UC1);
		cv::normalize(_cloud_proj->noise_image(), mono16_img, 0.0, 65535.0, cv::NORM_MINMAX, CV_16UC1);
		cv::resize(mono16_img, mono16_img, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
		sensor_msgs::ImagePtr noise_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, mono16_img).toImageMsg();
		_pub_NoiseImage.publish(noise_img);
	}

	// //save the generated depth image
	// std::string filename = std::string("depth_image_from_cloud");
	// filename += std::string("_") + timeToStr(ros::WallTime::now()) + std::string(".png");
	// CloudProjection::cvMatToDepthPNG(_cloud_proj->depth_image(), filename);
}

template<class T>
std::string CloudToImage::timeToStr(T ros_t)
{
    (void)ros_t;
    std::stringstream msg;
    const boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_facet *const f = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S.%f");
    msg.imbue(std::locale(msg.getloc(),f));
    msg << now;
    return msg.str();
}

}
