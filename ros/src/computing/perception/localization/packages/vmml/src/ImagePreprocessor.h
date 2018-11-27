/*
 * ImagePreprocessor.h
 *
 *  Created on: Nov 28, 2018
 *      Author: sujiwo
 */

#ifndef VMML_SRC_IMAGEPREPROCESSOR_H_
#define VMML_SRC_IMAGEPREPROCESSOR_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class ImagePreprocessor
{
public:

	enum ProcessMode {
		AS_IS = 0,
		AGC = 1,
		ILLUMINATI = 2
	};

	ImagePreprocessor(ProcessMode m);
	virtual ~ImagePreprocessor();

	cv::Mat preprocess(const cv::Mat &src);
	void preprocess(cv::Mat &srcInplace);

protected:

	ProcessMode pMode;
	cv::Mat mask;

	float iAlpha;
};


cv::Mat histogram (cv::Mat &inputMono, cv::Mat mask=cv::Mat());

#endif /* VMML_SRC_IMAGEPREPROCESSOR_H_ */
