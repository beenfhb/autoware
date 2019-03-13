/*
 * Trajectory.cpp
 *
 *  Created on: Dec 5, 2018
 *      Author: sujiwo
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "Trajectory.h"


using namespace std;


#define dPrecision 3


class PoseDumpStream: public std::stringstream
{
public:
PoseDumpStream(int sprecision=dPrecision)
{
	setf(ios::fixed, ios::floatfield);
	precision(sprecision);
}
};



PoseStamped
PoseStamped::operator* (const Pose &transform)
{
//	Pose me = Pose::from_Pos_Quat(this->position(), this->orientation());
	Pose P = static_cast<Pose&>(*this) * transform;
	return PoseStamped(P, this->timestamp);
}


PoseStamped
PoseStamped::interpolate(
	const PoseStamped &p1,
	const PoseStamped &p2,
	const ptime &t)
{
	assert (p1.timestamp<=t and t<=p2.timestamp);
	double r = toSeconds(t - p1.timestamp) / toSeconds(p2.timestamp - p1.timestamp);
	return PoseStamped (Pose::interpolate(p1, p2, r), t);
}


Quaterniond
PoseStamped::extrapolate(const PoseStamped &p1, const PoseStamped &p2, const decltype(PoseStamped::timestamp) &txinp)
{
	assert (p1.timestamp < p2.timestamp);
	if (p1.timestamp <= txinp and txinp <= p2.timestamp)
		throw runtime_error("Requested timestamp are within both data points; use interpolation instead");

	Vector3d euler1, euler2, eulerx;
	euler1 = quaternionToRPY(p1.orientation());
	euler2 = quaternionToRPY(p2.orientation());
	double t1, t2, tx, r;
	t1 = toSeconds(p1.timestamp);
	t2 = toSeconds(p2.timestamp);
	tx = toSeconds(txinp);

	if (txinp<=p1.timestamp) {
		r = (t1 - tx) / (t2 - t1);
		eulerx.x() = euler1.x() - r*(euler2.x() - euler1.x());
		eulerx.y() = euler1.y() - r*(euler2.y() - euler1.y());
		eulerx.z() = euler1.z() - r*(euler2.z() - euler1.z());
	}

	else {
		r = (tx - t1) / (t2 - t1);
		eulerx.x() = euler1.x() + r*(euler2.x() - euler1.x());
		eulerx.y() = euler1.y() + r*(euler2.y() - euler1.y());
		eulerx.z() = euler1.z() + r*(euler2.z() - euler1.z());
	}

//	eulerx.x() = normalizeAngle(eulerx.x());
//	eulerx.y() = normalizeAngle(eulerx.y());
//	eulerx.z() = normalizeAngle(eulerx.z());

	return fromRPY(eulerx.x(), eulerx.y(), eulerx.z());
}


string
PoseStamped::dump()
const
{
	PoseDumpStream s;

	s << toSeconds(timestamp) << ' ';
	Vector3d v = position();
	s << v.x() << " " << v.y() << " " << v.z() << " ";
	Quaterniond q = orientation();
	s << q.x() << " " << q.y() << " " << q.z() << ' ' << q.w();

	return s.str();
}


void
Trajectory::push_back(const PoseStamped &pt)
{
	if (size()>0)
		assert(pt.timestamp >= back().timestamp);
	return Parent::push_back(pt);
}


uint32_t
Trajectory::find_lower_bound(const ptime &t) const
{
	if ( t < front().timestamp or t > back().timestamp )
		throw out_of_range("Time out of range");

	auto it = std::lower_bound(begin(), end(), t,
		[](const PoseStamped &el, const ptime tv)
			-> bool {return el.timestamp < tv;}
	);
	uint32_t x = it-begin();
	return x;
}


/*
uint32_t
Trajectory::find_lower_bound(const ros::Time &t) const
{
	ptime tx = t.toBoost();
	return find_lower_bound(tx);
}
*/


PoseStamped
Trajectory::at(const ptime &t) const
{

	return Parent::at(find_lower_bound(t));
}


PoseStamped
Trajectory::interpolate (const ptime& t) const
{
	assert (front().timestamp<=t and t < back().timestamp);

	uint32_t i0 = find_lower_bound(t)-1;
	uint32_t i1 = i0+1;
	return PoseStamped::interpolate(Parent::at(i0), Parent::at(i1), t);
}


PoseStamped
Trajectory::extrapolate (const ptime& t) const
{
	assert (t < front().timestamp or t > back().timestamp);

	double r;
	Vector3d xpos;
	Quaterniond xori;

	if (t > back().timestamp) {
		size_t sz = size();
		const PoseStamped
			&lastPose = back(),
			&lastPose_1 = Parent::at(sz-2);

		r = (toSeconds(t) - toSeconds(lastPose_1.timestamp)) /
			(toSeconds(lastPose.timestamp) - toSeconds(lastPose_1.timestamp));

		xpos = lastPose_1.position() + r*(lastPose.position() - lastPose_1.position());
		xori = PoseStamped::extrapolate(lastPose_1, lastPose, t);
	}

	else if (t < front().timestamp) {
		const PoseStamped
			&firstPose = front(),
			&firstPose_1 = Parent::at(1);

		r = (toSeconds(t) - toSeconds(firstPose_1.timestamp)) /
			(toSeconds(firstPose.timestamp) - toSeconds(firstPose_1.timestamp));
		xpos = firstPose_1.position() + r*(firstPose.position() - firstPose_1.position());
		xori = PoseStamped::extrapolate(firstPose, firstPose_1, t);
	}

	return PoseStamped(xpos, xori, t);
}


Trajectory
Trajectory::subset(const ptime &start, const ptime &stop) const
{
	assert(start>=front().timestamp and stop<=back().timestamp);

	Trajectory ssub;
	for (auto it=begin(); it!=end(); ++it) {
		auto &p = *it;
		if (start<=p.timestamp and p.timestamp<=stop)
			ssub.push_back(p);
	}

	return ssub;
}


bool
Trajectory::dump(const std::string &filename) const
{
	fstream dsTrFd (filename, ios_base::out|ios_base::trunc);
	if (!dsTrFd.is_open()) {
		throw runtime_error("Unable to create "+filename);
	}

	for (int i=0; i<size(); ++i) {
		dsTrFd << at(i).dump() << endl;
	}

	dsTrFd.close();

	return true;
}


