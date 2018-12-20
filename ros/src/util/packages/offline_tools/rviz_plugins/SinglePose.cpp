/*
 * SinglePose.cpp
 *
 *  Created on: Dec 19, 2018
 *      Author: sujiwo
 */

#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <rviz_plugins/SinglePose.h>
#include <rviz/display.h>

#include <tf/tf.h>


using rviz::MovableText;


SinglePose::SinglePose()
{
	displayName = new rviz::StringProperty("Name", "Me", "Identifier", this, SLOT(updatePoseValue()));

	Xv = new rviz::FloatProperty("X", 0, "X Value", this, SLOT(updatePoseValue()));
	Yv = new rviz::FloatProperty("Y", 0, "Y Value", this, SLOT(updatePoseValue()));
	Zv = new rviz::FloatProperty("Z", 0, "Z Value", this, SLOT(updatePoseValue()));
	rollv = new rviz::FloatProperty("Roll", 0, "Roll Value", this, SLOT(updatePoseValue()));
	pitchv = new rviz::FloatProperty("Pitch", 0, "Pitch Value", this, SLOT(updatePoseValue()));
	yawv = new rviz::FloatProperty("Yaw", 0, "Yaw Value", this, SLOT(updatePoseValue()));

	radiusv = new rviz::FloatProperty("Radius", 0.1, "Radius Value", this, SLOT(updatePoseValue()));
	sizev = new rviz::FloatProperty("Length", 1.0, "Length of Axes", this, SLOT(updatePoseValue()));
}


SinglePose::~SinglePose()
{
}


void
SinglePose::onInitialize()
{
	axesDisp = new rviz::Axes(
		scene_manager_,
		scene_node_,
		1.0, 0.1);

	poseName = new MovableText(displayName->getStdString(), "Liberation Sans", 0.1);
	poseName->setTextAlignment(MovableText::H_CENTER, MovableText::V_BELOW);
	// Put name slightly below the axes
	poseName->setLocalTranslation(Ogre::Vector3(0, -0.2, 0));
	poseName->setVisible(true);
	scene_node_->attachObject(poseName);

	redraw();
}


void
SinglePose::updatePoseValue()
{
	redraw();
}


void
SinglePose::redraw()
{
//	tf::
	axesDisp->setPosition(
		Ogre::Vector3(
			Xv->getFloat(),
			Yv->getFloat(),
			Zv->getFloat()));

	tf::Quaternion q;
	q.setRPY(rollv->getFloat(), pitchv->getFloat(), yawv->getFloat());
	axesDisp->setOrientation(Ogre::Quaternion(q.w(), q.x(), q.y(), q.z()));

	axesDisp->set(sizev->getFloat(), radiusv->getFloat());

	poseName->setCaption(displayName->getStdString());

	queueRender();
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SinglePose, rviz::Display);
