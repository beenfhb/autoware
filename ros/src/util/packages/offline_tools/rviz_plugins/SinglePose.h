/*
 * SinglePose.h
 *
 *  Created on: Dec 19, 2018
 *      Author: sujiwo
 */

#ifndef _SINGLEPOSE_H_
#define _SINGLEPOSE_H_

#include <rviz/ogre_helpers/axes.h>
#include <rviz/display.h>
#include <rviz/properties/float_property.h>


class SinglePose : public rviz::Display
{
public:

Q_OBJECT

	SinglePose();
	virtual ~SinglePose();

private
	Q_SLOTS:

protected:
	virtual void onInitialize();
	void redraw();
};

#endif /* _SINGLEPOSE_H_ */
