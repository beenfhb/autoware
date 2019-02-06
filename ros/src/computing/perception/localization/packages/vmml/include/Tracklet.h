/*
 * Tracklet.h
 *
 *  Created on: Feb 6, 2019
 *      Author: sujiwo
 */

#ifndef _TRACKLET_H_
#define _TRACKLET_H_

#include <vector>
#include <map>
#include "BaseFrame.h"
#include "utilities.h"
#include "triangulation.h"


class VMap;


class Tracklet
{
public:
	Tracklet();

	virtual ~Tracklet();

	void insert (BaseFrame::Ptr frame, kpid k);

	Vector3d triangulate();

	cv::Mat getDescriptor() const;

	inline BaseFrame::ConstPtr getFirstFrame() const
	{ return *visibleFrames.begin(); }

	inline BaseFrame::ConstPtr getLastFrame() const
	{ return *visibleFrames.end(); }

	kpid getKeyPointFor (const BaseFrame::Ptr &frame) const;

	inline int size() const
	{ return visibleFrames.size(); }

	/*
	 * Find if the frame F has keypoint K in this tracklet
	 * Return values:
	 * 1: This tracklet does not have F
	 * 2: This tracklet has F, but not K
	 * 3: This tracklet has F and K
	 */
	int isHere (const BaseFrame::Ptr &F, kpid K) const;

protected:

	std::vector<BaseFrame::Ptr> visibleFrames;
	std::vector<kpid> keypoints;

	bool isTriangulated = false;
	Eigen::Vector3d position;
};

#endif /* _TRACKLET_H_ */
