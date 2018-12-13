
#include <QString>

#include "BagViewer.h"
#include "ui_BagViewer.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

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

		if (vi.second=="sensor_msgs/Image") {
			RandomAccessBag::Ptr smImg (new RandomAccessBag(*bagFdPtr, vi.first));
			imageBagList.push_back(smImg);
			ui->topicSelector->addItem(QString(vi.first.c_str()));
		}
	}

	currentActiveTopic = imageBagList.at(0);
}
