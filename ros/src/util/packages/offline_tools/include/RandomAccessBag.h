/*
 * RandomAccessBag.h
 *
 *  Created on: Aug 15, 2018
 *      Author: sujiwo
 */

#ifndef _RANDOMACCESSBAG_H_
#define _RANDOMACCESSBAG_H_


#include <string>
#include <vector>
#include <utility>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <memory>
#include <exception>


class time_exception: public std::runtime_error
{
	virtual const char* what() const throw()
	{ return "Invalid out-of-time requested"; }
};


class RandomAccessBag: public rosbag::View {
public:

	typedef std::shared_ptr<RandomAccessBag> Ptr;

	RandomAccessBag(
		const rosbag::Bag &bag, const std::string &topic);

	RandomAccessBag(
		const rosbag::Bag &bag, const std::string &topic,
		const ros::Time &t1, const ros::Time &t2);

	RandomAccessBag(
		rosbag::Bag const &bag, const std::string &topic,
		const double seconds1FromOffset,
		const double seconds2FromOffset);

	virtual ~RandomAccessBag();

	void setTimeConstraint(const double seconds1FromOffset, const double seconds2FromOffset);
	void setTimeConstraint(const ros::Time &t1, const ros::Time &t2);

	template<typename T>
	boost::shared_ptr<T>
	at (int position)
	{
		assert(position>=0 and position<size());
		return instantiate<T>(msgPtr.at(position));
	}

	RandomAccessBag subset(const ros::Time &start, ros::Duration &d) const;

	ros::Time timeAt (const int i) const
	{
		return msgPtr.at(i).time;
	}

	template<typename T>
	boost::shared_ptr<T>
	atDurationSecond (const double S)
	{
		return at<T> (getPositionAtDurationSecond(S));
	}

	std::string getTopic ()
	{ return conn->topic; }

	size_t size() const
	{ return static_cast<size_t>(size_cache_); }

	uint32_t getPositionAtDurationSecond (const double S) const;

	uint32_t getPositionAtTime (const ros::Time &tx) const;

	/*
	 * Duration of this view in ros::Duration
	 */
	 ros::Duration length() const
	 { return stopTime()-startTime(); }

	 ros::Time startTime() const
	 { return msgPtr.front().time; }

	 ros::Time stopTime() const
	 { return msgPtr.back().time; }

	 /*
	  * Convert time as represented by seconds from offset
	  */
	 ros::Time timeFromOffset(const double secondsFromStart) const;

protected:
	void createCache();

	const rosbag::Bag &bagstore;
	const rosbag::ConnectionInfo* conn;
	std::vector<rosbag::IndexEntry> msgPtr;

	template<class T>
	boost::shared_ptr<T>
	instantiate (const rosbag::IndexEntry &index_entry)
	{
		rosbag::MessageInstance *m = newMessageInstance(conn, index_entry, bagstore);
		return m->instantiate<T>();
	}

	ros::Time
		bagStartTime,
		bagStopTime;
	const std::string viewTopic;

};

#endif /* _RANDOMACCESSBAG_H_ */
