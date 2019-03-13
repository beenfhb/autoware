/*
 * Trajectory.h
 *
 *  Created on: Dec 5, 2018
 *      Author: sujiwo
 */

#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_


#include <vector>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/date_time/posix_time/time_serialize.hpp>

#include <ros/time.h>

#include "utilities.h"


struct PoseStamped : public Pose
{
	ptime timestamp;

	PoseStamped():
		Pose()
	{ timestamp = ros::Time(0).toBoost(); }

	PoseStamped(const Pose &p, const ptime &t=unixTime0)
	{
		m_matrix = p.matrix();
		timestamp = t;
	}

	inline PoseStamped (const Eigen::Vector3d &p, const Quaterniond &q, const ptime &t=unixTime0)
	{
		m_matrix = Pose::from_Pos_Quat(p, q).matrix();
		timestamp = t;
	}

	PoseStamped operator* (const Pose &t);

	inline double timeSecond () const
	{ return toSeconds(timestamp); }

	static PoseStamped interpolate(
		const PoseStamped &p1,
		const PoseStamped &p2,
		const ptime &t);

	static
	Quaterniond
	extrapolate(const PoseStamped &p1, const PoseStamped &p2, const decltype(PoseStamped::timestamp) &tx);

	template<class Archive>
	inline void save(Archive &ar, const unsigned int v) const
	{
		// Eigen matrix
		ar << boost::serialization::base_object<Pose>(*this);
		ar << timestamp;
	}

	template<class Archive>
	inline void load(Archive &ar, const unsigned int v)
	{
		// Eigen matrix
		ar >> boost::serialization::base_object<Pose>(*this);
		ar >> timestamp;
	}

	std::string dump() const;

	BOOST_SERIALIZATION_SPLIT_MEMBER()
};





class Trajectory : public std::vector<PoseStamped>
{
public:

	friend class boost::serialization::access;

	void push_back(const PoseStamped &);

	// Return nearest element of provided time
	PoseStamped at(const ptime&) const;

	PoseStamped at(const int idx) const
	{ return std::vector<PoseStamped>::at(idx); }

	PoseStamped interpolate (const ptime&) const;

	PoseStamped extrapolate (const ptime&) const;

	Trajectory subset(const ptime &start, const ptime &stop) const;

	bool dump(const std::string &filename) const;

private:
	uint32_t
	find_lower_bound(const ptime&) const;

/*
	uint32_t
	find_lower_bound(const ros::Time&) const;
*/

	typedef std::vector<PoseStamped> Parent;

	template<class Archive>
	inline void serialize(Archive &ar, const unsigned int version)
	{ ar & boost::serialization::base_object<Parent>(*this);}

};

#endif /* _TRAJECTORY_H_ */
