/*
 * Tracklet.cpp
 *
 *  Created on: Feb 6, 2019
 *      Author: sujiwo
 */

#include "Tracklet.h"
#include "VMap.h"

#include <exception>

using namespace std;


Tracklet::Tracklet()
{}


Tracklet::~Tracklet()
{}


void
Tracklet::insert (BaseFrame::Ptr frame, kpid k)
{
	visibleFrames.push_back(frame);
	keypoints.push_back(k);
}


kpid
Tracklet::getKeyPointFor (const BaseFrame::Ptr &frame) const
{
	for (uint i=0; i<visibleFrames.size(); ++i) {
		if (frame==visibleFrames.at(i))
			return keypoints.at(i);
	}

	throw std::out_of_range("Invalid pointer key");
}


int
Tracklet::isHere (const BaseFrame::Ptr &F, kpid K) const
{
	try {
		kpid kx = getKeyPointFor(F);
		if (kx==K)
			return 3;
		else
			return 2;
	} catch (out_of_range &e) {
		return 1;
	}
}
