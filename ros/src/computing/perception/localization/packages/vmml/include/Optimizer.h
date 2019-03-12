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

#include <g2o/types/sba/types_six_dof_expmap.h>

#include "KeyFrame.h"
#include "MapPoint.h"


class Frame;


class G2O_TYPES_SBA_API VertexCameraMono : public g2o::VertexSE3Expmap
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexCameraMono() : g2o::VertexSE3Expmap() {}

	void set(KeyFrame *_f);

	void updateToMap()
	{ return kf->setPose(estimate()); }

	kfid getIdFromMap () const
	{ return kf->getId(); }

	KeyFrame *kf = nullptr;
};


class G2O_TYPES_SBA_API VertexMapPoint : public g2o::VertexSBAPointXYZ
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VertexMapPoint() : g2o::VertexSBAPointXYZ() {}

	void set(MapPoint *_mp);

	void updateToMap()
	{ return mp->setPosition(estimate()); }

	mpid getIdFromMap() const
	{ return mp->getId(); }

	kpid getKeyPointId(const KeyFrame &k) const
	{
		return k.getParent().getKeyPointId(k.getId(), getIdFromMap());
	}

	kpid getKeyPointId(const VertexCameraMono *k) const
	{
		return getKeyPointId(*k->kf);
	}

	MapPoint *mp = nullptr;
};


class G2O_TYPES_SBA_API EdgeProjectMonocular : public g2o::EdgeProjectXYZ2UV
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeProjectMonocular() : g2o::EdgeProjectXYZ2UV() {}

	/*
	 * Perform multiple jobs: set estimation and information matrix
	 */
	void set(VertexCameraMono *_f, VertexMapPoint *_p);
	Vector3d transformWorldPointToFrame(const Vector3d &pointInWorld) const;
	bool isDepthPositive() const;
};


class G2O_TYPES_SBA_API EdgeFrameMovement : public g2o::EdgeSE3Expmap
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeFrameMovement() : g2o::EdgeSE3Expmap() {}
	void set(VertexCameraMono &f1, VertexCameraMono &f2);
};


void bundle_adjustment (VMap *orgMap);

void bundle_adjustment_2 (VMap *orgMap);

int optimize_pose (const Frame &frame, Pose &initPose, const VMap *vmap);

void local_bundle_adjustment (VMap *origMap, const kfid &targetKf);


#endif /* OPTIMIZER_H_ */
