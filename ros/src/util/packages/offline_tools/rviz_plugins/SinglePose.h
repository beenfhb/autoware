/*
 * SinglePose.h
 *
 *  Created on: Dec 19, 2018
 *      Author: sujiwo
 */

#ifndef _SINGLEPOSE_H_
#define _SINGLEPOSE_H_

#include <memory>

#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <rviz/display.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>


class SinglePose : public rviz::Display
{
Q_OBJECT
public:

	SinglePose();
	virtual ~SinglePose();

private
	Q_SLOTS:
		void updatePoseValue();

protected:
	virtual void onInitialize();
	void redraw();

	rviz::StringProperty *displayName;

	rviz::FloatProperty
		*Xv,
		*Yv,
		*Zv,
		*rollv,
		*pitchv,
		*yawv,
		*radiusv,
		*sizev;

	rviz::Axes *axesDisp = nullptr;
	rviz::MovableText *poseName = nullptr;
};

#endif /* _SINGLEPOSE_H_ */
