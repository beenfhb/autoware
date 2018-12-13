
#include <iostream>

#include <QString>

#include "BagViewer.h"
#include "ui_BagViewer.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>

using namespace std;


BagViewer::BagViewer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::BagViewer)
{
    ui->setupUi(this);
}


BagViewer::~BagViewer()
{
    delete ui;
}


void
BagViewer::setBagFile(const std::string &bagfilename)
{
	bagFdPtr = std::shared_ptr<rosbag::Bag>(new rosbag::Bag(bagfilename, rosbag::BagMode::Read));

	ui->topicSelector->clear();

	auto vTopicList = RandomAccessBag::getTopicList(*bagFdPtr);
	for (auto vi: vTopicList) {

		if (vi.second=="sensor_msgs/Image" or vi.second=="sensor_msgs/CompressedImage") {
			RandomAccessBag::Ptr smImg (new RandomAccessBag(*bagFdPtr, vi.first));
			imageBagList.push_back(smImg);
			ui->topicSelector->addItem(QString(vi.first.c_str()));
		}
	}

	setTopic(0);
}


void
BagViewer::setTopic(int n)
{
	if (n>=imageBagList.size())
		return;

	currentActiveTopic = imageBagList.at(n);
	ui->playProgress->setRange(0, currentActiveTopic->size()-1);

	updateImage(0);
}


void
BagViewer::on_playButton_clicked(bool checked)
{
	cerr << "Play Clicked\n";
}


void
BagViewer::on_playProgress_sliderMoved(int i)
{
	updateImage(i);
}


void
BagViewer::on_topicSelector_currentIndexChanged(int i)
{
	setTopic(i);
}


void
BagViewer::updateImage(int n)
{
	if (n>=currentActiveTopic->size())
		return;

	if (currentActiveTopic->messageType()=="sensor_msgs/Image") {
		sensor_msgs::Image::ConstPtr imageMsg = currentActiveTopic->at<sensor_msgs::Image>(n);
		currentImage = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::RGB8)->image;

	}

	else if (currentActiveTopic->messageType()=="sensor_msgs/CompressedImage") {
		sensor_msgs::CompressedImage::ConstPtr imageMsg = currentActiveTopic->at<sensor_msgs::CompressedImage>(n);
		currentImage = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::RGB8)->image;
	}

	QImage curImage (currentImage.data, currentImage.cols, currentImage.rows, currentImage.step[0], QImage::Format_RGB888);
	ui->imageFrame->setImage(curImage);
}
