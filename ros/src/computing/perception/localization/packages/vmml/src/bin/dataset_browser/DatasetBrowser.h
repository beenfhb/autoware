/*
 * DatasetBrowser.h
 *
 *  Created on: Aug 9, 2018
 *      Author: sujiwo
 */

#ifndef _DATASETBROWSER_H_
#define _DATASETBROWSER_H_

#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QPushButton>

#include "MTContainers.h"

#include "ui_dataset_browser.h"
#include "ratio_layouted_frame.h"

#include "BaseFrame.h"
#include "datasets/GenericDataset.h"
#include "datasets/LidarScanBag.h"
#include "datasets/OxfordDataset.h"
#include "datasets/MeidaiBagDataset.h"

#include <pcl/point_types.h>


class DatasetBrowser: public QWidget
{
Q_OBJECT

public:

	explicit DatasetBrowser(QWidget *parent=NULL);
	virtual ~DatasetBrowser();

	enum datasetType {
		OxfordType,
		MeidaiType
	};

	void changeDataset(GenericDataset::Ptr ds, datasetType t);

public slots:
	void on_timelineSlider_sliderMoved(int value);
	void on_saveImageButton_clicked(bool checked);
	void on_playButton_clicked(bool checked);
	void on_nextFrameButton_clicked(bool c);
	void on_prevFrameButton_clicked(bool c);
	void on_preprocessImageCheck_stateChanged(int s);
	void on_frame_mouseMove(int x, int y);
	void on_scaleInput_editingFinished();
	void on_pcdFileChooser_clicked(bool c);
	void on_pcdCheckShow_stateChanged(int s);
	void on_savePcdButton_clicked(bool checked);

private:
	Ui::DatasetBrowser_frm ui;

	GenericDataset::Ptr openDs;
	// Cast pointer
	MeidaiBagDataset::Ptr meidaiDs;
	OxfordDataset::Ptr oxfordDs;

	CameraPinholeParams datasetCameraParam;

	QSlider *timelineSlider;
	RatioLayoutedFrame *frame;
	QLabel *timeOffsetLabel;
	QPushButton *saveImageButton;
	QCheckBox *enableLidarScanRender;
	QPlainTextEdit *frameTextInfo;

	QPushButton *playButton;

private:
	void setImageOnPosition (int v);
	void startPlayBag();
	void stopPlayBag();
	void disableControlsOnPlaying (bool state);

	std::vector<BaseFrame::PointXYI> projectScan
	(pcl::PointCloud<pcl::PointXYZ>::ConstPtr lidarScan)
	const;

	static void drawPoints (cv::Mat &target, const std::vector<BaseFrame::PointXYI> &pointList);
	static void drawPoints (cv::Mat &target, const BaseFrame::MatrixProjectionResult &pointM);

	GenericDataItem::ConstPtr dataItem0;

	// Special case for Meidai Bag Dataset: show projection of Velodyne scans
	LidarScanBag::Ptr meidaiPointClouds = nullptr;

	std::string getCurrentFrameInfo(uint32_t frNum) const;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudMap = nullptr;

};

#endif /* _DATASETBROWSER_H_ */
