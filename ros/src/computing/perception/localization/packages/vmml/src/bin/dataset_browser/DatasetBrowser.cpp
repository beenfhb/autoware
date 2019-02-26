/*
 * DatasetBrowser.cpp
 *
 *  Created on: Aug 9, 2018
 *      Author: sujiwo
 */

#include <iostream>
#include <sstream>
#include <string>
#include <chrono>
#include <thread>
#include <functional>
#include <boost/filesystem.hpp>

#include <opencv2/core.hpp>
#include <SequenceSLAM.h>
#include <QImage>
#include <QFileDialog>

#include "DatasetBrowser.h"
#include "BaseFrame.h"

#include "datasets/MeidaiBagDataset.h"


using namespace std;

using Path = boost::filesystem::path;


// XXX: Find a way to specify these values from external input
CameraPinholeParams meidaiCamera1Params(
	1150.96938467,	// fx
	1150.96938467,	// fy
	988.511326762,	// cx
	692.803953253,	// cy
	1920,			// width
	1440			// height
);



DatasetBrowser::DatasetBrowser(QWidget *parent):
	QWidget(parent)
{
	ui.setupUi(this);

	timelineSlider = ui.timelineSlider;
	frame = ui.frame;
	timeOffsetLabel = ui.timeOffsetLabel;
	saveImageButton = ui.saveImageButton;
	playButton = ui.playButton;
	enableLidarScanRender = ui.enableLidarScanRender;
	frameTextInfo = ui.frameTextInfo;
}

DatasetBrowser::~DatasetBrowser()
{}


void
DatasetBrowser::on_timelineSlider_sliderMoved(int v)
{
	return setImageOnPosition(v);
}

const string defaultImageExtension = "png";

void
DatasetBrowser::on_saveImageButton_clicked(bool checked)
{
	Path cwd = boost::filesystem::current_path();
	int currentId = timelineSlider->value();
	string imageName = to_string(currentId) + '.' + defaultImageExtension;
	Path fullName = cwd / Path(imageName);

	QString fname = QFileDialog::getSaveFileName(this, tr("Save Image"), QString::fromStdString(fullName.string()));
	if (fname.length()==0)
		return;
	cv::Mat image = openDs->get(timelineSlider->value())->getImage();
	cv::imwrite(fname.toStdString(), image);
}


const string defaultPointCloudExtension = "pcd";

void
DatasetBrowser::on_savePcdButton_clicked(bool checked)
{
	Path cwd = boost::filesystem::current_path();
	int currentId = timelineSlider->value();
	string imageName = to_string(currentId) + '.' + defaultPointCloudExtension;
	Path fullName = cwd / Path(imageName);

	QString fname = QFileDialog::getSaveFileName(this, tr("Save Point Cloud"), QString::fromStdString(fullName.string()));
	if (fname.length()==0)
		return;

	auto pcd = meidaiDs->getNative(timelineSlider->value())->getLidarScan();
	LidarScanBag::save(pcd, fname.toStdString());
}


std::string dPoseLean (const Pose &frame)
{
	stringstream ss;
	ss << fixed << setprecision(6);
	auto P = frame.position();
	auto Q = frame.orientation();

	ss << P.x() << ' ' << P.y() << ' ' << P.z() << ' ';
	ss << Q.x() << ' ' << Q.y() << ' ' << Q.z() << ' ' << Q.w();

	return ss.str();
}


// XXX: Change this
const string lidarCalibrationParams("/home/sujiwo/Autoware/ros/src/computing/perception/localization/packages/vmml/params/meidai-64e-S2.yaml");

void
DatasetBrowser::changeDataset(GenericDataset::Ptr ds, datasetType ty)
{
	openDs = ds;
	timelineSlider->setRange(0, ds->size()-1);
	dataItem0 = ds->get(0);

	if (ty==DatasetBrowser::MeidaiType) {
		meidaiDs = static_pointer_cast<MeidaiBagDataset>(ds);
		meidaiDs->setLidarParameters(lidarCalibrationParams, string(), defaultLidarToCameraTransform);
		meidaiPointClouds = meidaiDs->getLidarScanBag();
		meidaiDs->isPreprocessed = true;
		ui.preprocessImageCheck->setChecked(meidaiDs->isPreprocessed);
		meidaiDs->addCameraParameter(meidaiCamera1Params);
	}
	else if (ty==DatasetBrowser::OxfordType) {
		oxfordDs = static_pointer_cast<OxfordDataset>(ds);
		// Can not be used in Oxford dataset
		ui.enableLidarScanRender->setDisabled(true);
		ui.preprocessImageCheck->setDisabled(true);
	}

	datasetCameraParam = openDs->getCameraParameter();
	ui.scaleInput->setText(QString("%1").arg(openDs->getZoomRatio(), 0, 'f', 3));

	setImageOnPosition(0);
}


bool isTimeInside (const LidarScanBag::Ptr &bg, ros::Time Tx)
{
	return (Tx>=bg->startTime() and Tx<bg->stopTime());
}


const double pcdMapFarDistance = 100;

void
DatasetBrowser::setImageOnPosition (int v)
{
	if (v<0 or v>=openDs->size())
		throw runtime_error("Invalid time position");

	auto curItem = openDs->get(v);

	auto ts = curItem->getTimestamp() - dataItem0->getTimestamp();
	double tsd = double(ts.total_microseconds())/1e6;

	stringstream ss;
	ss << fixed << setprecision(2) << tsd << "(" << to_string(v) << ")";
	timeOffsetLabel->setText(QString::fromStdString(ss.str()));

	cv::Mat image = curItem->getImage();
	cv::cvtColor(image, image, CV_BGR2RGB);

	try {
		if (enableLidarScanRender->isChecked()) {

			auto imageTime = ros::Time::fromBoost(curItem->getTimestamp());
			if (meidaiPointClouds!=nullptr and isTimeInside(meidaiPointClouds, imageTime)) {

				uint32_t pcIdx = meidaiPointClouds->getPositionAtTime(imageTime);
				auto pointCloud = meidaiPointClouds->at(pcIdx);
				auto projections = projectScan(pointCloud);

				drawPoints(image, projections);
			}
		}
	} catch (const std::exception &e) {}

	try {
		if (ui.pcdCheckShow->isChecked() and pointCloudMap!=nullptr) {
			BaseFrame::MatrixProjectionResult pcdmapProj;
			BaseFrame::projectPointCloud(pointCloudMap, datasetCameraParam, curItem->getPose(), pcdMapFarDistance, pcdmapProj);
			drawPoints(image, pcdmapProj);
		}
	} catch (const std::exception &e) {}

	QImage curImage (image.data, image.cols, image.rows, image.step[0], QImage::Format_RGB888);
	frame->setImage(curImage);

//	frameTextInfo->setPlainText(QString::fromStdString(getCurrentFrameInfo(v)));
	QString frameInfoStr = QString::fromStdString(getCurrentFrameInfo(v));
	QMetaObject::invokeMethod(frameTextInfo, "setPlainText", Q_ARG(QString, frameInfoStr));
}


void
DatasetBrowser::disableControlsOnPlaying (bool state)
{
	timelineSlider->setDisabled(state);
	saveImageButton->setDisabled(state);
	ui.nextFrameButton->setDisabled(state);
	ui.prevFrameButton->setDisabled(state);
	ui.savePcdButton->setDisabled(state);
}


void
DatasetBrowser::on_playButton_clicked(bool checked)
{
	static bool playStarted = false;
	static std::thread *playerThread = NULL;

	std::function<void()> playThreadFn =
	[&]()
	{
		const int startPos = timelineSlider->sliderPosition();
		disableControlsOnPlaying(true);
		for (int p=startPos; p<=timelineSlider->maximum(); p++) {

			ptime t1x = getCurrentTime();
			timelineSlider->setSliderPosition(p);
			setImageOnPosition(p);
			if (playStarted == false)
				break;

			if(p < timelineSlider->maximum()) {
				ptime t1 = openDs->get(p)->getTimestamp();
				ptime t2 = openDs->get(p+1)->getTimestamp();
				ptime t2x = getCurrentTime();
				tduration tdx = t2x - t1x;	// processing overhead
				tduration td = (t2-t1) - tdx;
				std::this_thread::sleep_for(std::chrono::milliseconds(td.total_milliseconds()));
			}
		}
		disableControlsOnPlaying(false);
	};

	if (checked==true) {
		playStarted = true;
		playerThread = new std::thread(playThreadFn);
	}

	else {
		playStarted = false;
		playerThread->join();
		delete(playerThread);
	}

	return;
}


void
DatasetBrowser::on_nextFrameButton_clicked(bool c)
{
	int curPos = timelineSlider->sliderPosition();
	if (curPos < timelineSlider->maximum()) {
		curPos += 1;
		timelineSlider->setSliderPosition(curPos);
		setImageOnPosition(curPos);
	}
}


void
DatasetBrowser::on_prevFrameButton_clicked(bool c)
{
	int curPos = timelineSlider->sliderPosition();
	if (curPos > 0) {
		curPos -= 1;
		timelineSlider->setSliderPosition(curPos);
		setImageOnPosition(curPos);
	}
}


void
DatasetBrowser::on_preprocessImageCheck_stateChanged(int state)
{
	state==0 ?
		meidaiDs->isPreprocessed=false :
		meidaiDs->isPreprocessed=true;
	setImageOnPosition(timelineSlider->sliderPosition());
}


void
DatasetBrowser::on_frame_mouseMove(int frame_x, int frame_y)
{
	double xr, yr;
	xr = (double(datasetCameraParam.width) / double(frame->width())) * double(frame_x);
	yr = (double(datasetCameraParam.height) / double(frame->height())) * double(frame_y);
	ui.xCoordPos->setText(QString("%1").arg(xr, 0, 'f', 3));
	ui.yCoordPos->setText(QString("%1").arg(yr, 0, 'f', 3));
}


void
DatasetBrowser::on_scaleInput_editingFinished()
{
	double scale = ui.scaleInput->text().toDouble();
	openDs->setZoomRatio(scale);
	datasetCameraParam = openDs->getCameraParameter();
	setImageOnPosition(timelineSlider->sliderPosition());
}


std::vector<BaseFrame::PointXYI>
DatasetBrowser::projectScan
(pcl::PointCloud<pcl::PointXYZ>::ConstPtr lidarScan)
const
{
	return BaseFrame::projectLidarScan(lidarScan, defaultLidarToCameraTransform, datasetCameraParam);
}


const cv::Scalar projectionColor (255,0,0);


cv::Point2f convertPoint(const BaseFrame::PointXYI &pt)
{
	return cv::Point2f(pt.x, pt.y);
}


void
DatasetBrowser::drawPoints
(cv::Mat &target, const std::vector<BaseFrame::PointXYI> &pointList)
{
	for (auto &pt2d: pointList) {
		if ((pt2d.x>=0 and pt2d.x<target.cols) and (pt2d.y>=0 and pt2d.y<target.rows)) {
			cv::circle(target, convertPoint(pt2d), 2, projectionColor, -1);
		}
	}
}


void
DatasetBrowser::drawPoints (cv::Mat &target, const BaseFrame::MatrixProjectionResult &pointM)
{
	for (int r=0; r<pointM.rows(); ++r) {
		cv::Point2f p2f (pointM(r,0), pointM(r,1));
		cv::circle(target, p2f, 2, projectionColor, -1);
	}
}


std::string
DatasetBrowser::getCurrentFrameInfo(uint32_t frNum) const
{
	stringstream ss;
	ss << fixed << setprecision(3);

	auto curFrame = openDs->get(frNum);
	ss << "#: " << curFrame->getId() << endl;

	try {
	auto pos = curFrame->getPose();
		ss << "Position: " << pos.x() << ", " << pos.y() << ", " << pos.z() << endl;

		auto ort = curFrame->getOrientation();
		ss << "Orientation (XYZW): " << pos.qx() << ", " << pos.qy() << ", " << pos.qz() << ", " << pos.qw() << endl;

		auto rpyRad = quaternionToRPY(ort);
		ss << "Orientation (RPYrad): " << rpyRad[0] << ", " << rpyRad[1] << ", " << rpyRad[2] << endl;

	} catch (out_of_range &e) {
		ss << "This frame does not have position/orientation info" << endl;
	}

	return ss.str();
}


void
DatasetBrowser::on_pcdFileChooser_clicked(bool c)
{
	QString pcdFilename = QFileDialog::getOpenFileName(this, "Open PCD File");
	disableControlsOnPlaying(true);

	pointCloudMap = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader fReader;
	fReader.read(pcdFilename.toStdString(), *pointCloudMap);
	ui.pcdCheckShow->setDisabled(false);

	disableControlsOnPlaying(false);
}


void
DatasetBrowser::on_pcdCheckShow_stateChanged(int s)
{
	setImageOnPosition(timelineSlider->sliderPosition());
}
