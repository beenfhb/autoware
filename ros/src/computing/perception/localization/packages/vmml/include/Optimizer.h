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
#include "KeyFrame.h"
#include "MapPoint.h"


class Frame;

void bundle_adjustment (VMap *orgMap);

void bundle_adjustment_2 (VMap *orgMap);

int optimize_pose (const Frame &frame, Pose &initPose, const VMap *vmap);

void local_bundle_adjustment (VMap *origMap, const kfid &targetKf);


#endif /* OPTIMIZER_H_ */
