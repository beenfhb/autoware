#ifndef BAGVIEWER_H
#define BAGVIEWER_H

#include <vector>
#include <map>
#include <memory>

#include <QMainWindow>

#include <rosbag/bag.h>

#include <opencv2/opencv.hpp>

#include "RandomAccessBag.h"


namespace Ui {
class BagViewer;
}

class BagViewer : public QMainWindow
{
    Q_OBJECT

public:
	explicit BagViewer(QWidget *parent = 0);
	~BagViewer();

	void setBagFile(const std::string &bagfilename);

public slots:
	void on_playButton_clicked(bool checked);
	void on_playProgress_sliderMoved(int i);
	void on_topicSelector_currentIndexChanged(int i);

private:
    Ui::BagViewer *ui;

protected:
    // Store bag file descriptor object
	std::shared_ptr<rosbag::Bag> bagFdPtr = nullptr;
	// Currently active topic's bag view
	RandomAccessBag::Ptr currentActiveTopic = nullptr;

	std::vector<RandomAccessBag::Ptr> imageBagList;

	cv::Mat currentImage;

	void setTopic(int n);

	void updateImage(int n);
};

#endif // BAGVIEWER_H
