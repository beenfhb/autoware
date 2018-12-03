/*
 * RandomAccessBag.cpp
 *
 *  Created on: Aug 15, 2018
 *      Author: sujiwo
 */

#include <algorithm>
#include <exception>
#include <rosbag/query.h>

#include "RandomAccessBag.h"

#include "access_private.hpp"


using std::runtime_error;


ACCESS_PRIVATE_FIELD(rosbag::MessageInstance, rosbag::IndexEntry const, index_entry_);


/*
 * A Notes about ROS Bags:
 */


/*
 * Only allows one single topic
 */
RandomAccessBag::RandomAccessBag
	(const rosbag::Bag &_bag, const std::string &topic):

	bagstore(_bag),
	viewTopic(topic),
	rosbag::View::View(_bag, rosbag::TopicQuery(topic))

{
	bagStartTime = getBeginTime();
	bagStopTime = getEndTime();

	createCache();
}


RandomAccessBag::RandomAccessBag(
	const rosbag::Bag &_bag, const std::string &topic,
	const ros::Time &t1, const ros::Time &t2) :

	bagstore(_bag),
	viewTopic(topic),
	rosbag::View::View(_bag)

{
	bagStartTime = getBeginTime();
	bagStopTime = getEndTime();

	setTimeConstraint(t1, t2);
}



RandomAccessBag::RandomAccessBag(
	rosbag::Bag const &_bag, const std::string &topic,
	const double seconds1,
	const double seconds2) :

	bagstore(_bag),
	viewTopic(topic),
	rosbag::View::View(_bag)

{
	bagStartTime = getBeginTime();
	bagStopTime = getEndTime();

	setTimeConstraint(seconds1, seconds2);
}


RandomAccessBag::~RandomAccessBag()
{
}


void
RandomAccessBag::createCache()
{
	update();
	rosbag::View::size();
	iterator it = begin();
	size_t sz = this->size();
	conn = getConnections()[0];
	msgPtr.resize(sz);

	for (uint32_t p=0; p<sz; p++) {
		rosbag::MessageInstance &m = *it;
		rosbag::IndexEntry const ie = access_private::index_entry_(m);
		msgPtr.at(p) = ie;
		++it;
	}
}


void
RandomAccessBag::setTimeConstraint(
	const double seconds1,
	const double seconds2)
{
	assert (seconds1 <= seconds2);

	ros::Duration td1(seconds1),
		td2(seconds2);

	assert(bagStartTime + td2 <= bagStopTime);

	ros::Time
		t1 = bagStartTime + td1,
		t2 = bagStartTime + td2;

	setTimeConstraint(t1, t2);
}


void
RandomAccessBag::setTimeConstraint(const ros::Time &t1, const ros::Time &t2)
{
	if (t1 < bagStartTime or t1 > bagStopTime)
		throw runtime_error("Requested start time is out of range");

	if (t2 < bagStartTime or t2 > bagStopTime)
		throw runtime_error("Requested stop time is out of range");

	queries_.clear();
	ranges_.clear();
	addQuery(bagstore, rosbag::TopicQuery(viewTopic), t1, t2);
	createCache();
}


uint32_t
RandomAccessBag::getPositionAtDurationSecond (const double S) const
{
	ros::Duration Sd (S);
	ros::Time Tx = msgPtr.at(0).time + Sd;
	assert (Tx>= msgPtr.at(0).time and Tx<=msgPtr.back().time);

	return getPositionAtTime (Tx);
}


uint32_t
RandomAccessBag::getPositionAtTime (const ros::Time &tx) const
{
	if (tx<=msgPtr.at(0).time or tx>msgPtr.back().time)
		throw std::runtime_error("Specified time is outside the range");

	auto it = std::lower_bound(msgPtr.begin(), msgPtr.end(), tx,
		[](const rosbag::IndexEntry &iptr, const ros::Time &t)
		{ return (iptr.time < t); }
	);

	return it - msgPtr.begin();
}


/*
 * Convert time as represented by seconds from offset
 */
ros::Time
RandomAccessBag::timeFromOffset(const double secondsFromStart) const
{
	assert(secondsFromStart >= 0
		and secondsFromStart <= length().toSec());

	ros::Duration td(secondsFromStart);
	return msgPtr.at(0).time + td;
}

