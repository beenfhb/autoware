/*
 * optimizer.h
 *
 *  Created on: Jun 26, 2018
 *      Author: sujiwo
 */

#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include <map>
#include <set>
#include <vector>

#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include "KeyFrame.h"
#include "MapPoint.h"


class Frame;


class VertexCameraMono : public g2o::VertexSim3Expmap
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexCameraMono(KeyFrame &_f);

	KeyFrame &kf;
};


class VertexMapPoint : public g2o::VertexSBAPointXYZ
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexMapPoint(MapPoint &_mp);

	MapPoint &mp;
};


class EdgeProjectMonocular : public g2o::EdgeSim3ProjectXYZ
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeProjectMonocular();

	static Vector3d transformWorldPointToFrame(const g2o::SE3Quat &framePose, const Vector3d &pointInWorld);
	bool isDepthPositive() const;
};


void bundle_adjustment (VMap *orgMap);

void bundle_adjustment_2 (VMap *orgMap);

int optimize_pose (const Frame &frame, Pose &initPose, const VMap *vmap);

void local_bundle_adjustment (VMap *origMap, const kfid &targetKf);


#endif /* OPTIMIZER_H_ */
