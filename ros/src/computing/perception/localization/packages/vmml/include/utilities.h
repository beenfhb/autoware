/*
 * utilities.h
 *
 *  Created on: Jul 7, 2018
 *      Author: sujiwo
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_


#include <set>
#include <map>
#include <vector>
#include <utility>
#include <limits>
#include <algorithm>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/conversion.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/array.hpp>

#include <opencv2/core.hpp>

#include <ros/package.h>


using std::pair;
using std::set;
using std::map;
using std::vector;
using Eigen::Quaterniond;
using Eigen::Vector3d;


typedef uint64_t kfid;
typedef uint64_t mpid;
typedef decltype(cv::DMatch::trainIdx) kpid;


typedef boost::posix_time::ptime ptime;
typedef boost::posix_time::time_duration tduration;

const auto
	MIN_TIME = boost::posix_time::special_values::min_date_time,
	MAX_TIME = boost::posix_time::special_values::max_date_time;

const ptime
	unixTime0(boost::gregorian::date(1970,1,1));


template<typename P, typename Q>
map<Q,P> reverseMap (const map<P,Q> &smap)
{
	map<Q,P> nmap;
	for (auto &pq: smap) {
		P p = pq.first;
		Q q = pq.second;
		nmap[q] = p;
	}
	return nmap;
}


template<typename K, typename V>
set<K> allKeys (const map<K,V> &smap)
{
	set<K> retval;
	for (auto &p: smap) {
		retval.insert(p.first);
	}
	return retval;
}


template<typename T>
set<T> toSet (const vector<T> &vect)
{
	set<T> Res;
	for (T &v: vect) {
		Res.insert(v);
	}
	return Res;
}


template<typename T>
bool inSet (const set<T> &S, const T &val)
{
	return (S.find(val) != S.end());
}


// Result = A - B
template<typename T>
set<T> subtract (const set<T> &A, const set<T> &B)
{
	set<T> Res;
	for (auto &p: A) {
		if (!inSet(B, p)) {
			Res.insert(p);
		}
	}
	return Res;
}


template<typename K, typename V>
vector<V> allValues (const map<K,V> &sMap)
{
	vector<V> allV;
	for (auto &kv: sMap) {
		allV.push_back(kv.second);
	}
	return allV;
}


template<typename Scalar>
using VectorXx = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;


template <typename Scalar>
double median(const VectorXx<Scalar> &v)
{
	int n = v.rows();
	vector<Scalar> vs (v.data(), v.data()+v.rows());
	sort(vs.begin(), vs.end());
	if (n%2==1)
		return (double)vs[(n-1)/2];
	else
		return (double(vs[n/2])+double(vs[(n/2)-1])) / 2;
}


inline
int medianx (const Eigen::VectorXi &v)
{
	return median(v);
}


template <typename k, typename v>
pair<const k,v>
maximumMapElement(const map<k,v> &maptg)
{
	auto p = max_element(maptg.begin(), maptg.end(),
		[](const pair<k,v> &element1, const pair<k,v> &element2)
			{return element1.second < element2.second;}
	);
	return *p;
}


// Plane and Line manipulation
typedef Eigen::Hyperplane<double, 3> Plane3;
typedef Eigen::Hyperplane<double, 2> Line2;

// Lines in 3D are represented as parametrized lines
typedef Eigen::ParametrizedLine<double, 3> Line3;


/*
 * All angles are in Radian
 */
Quaterniond fromRPY (double roll, double pitch, double yaw);

Vector3d quaternionToRPY (const Quaterniond &q);

class TQuaternion : public Eigen::Quaterniond
{
public:
	inline TQuaternion(const double &x, const double &y, const double &z, const double &w):
		Quaterniond(w, x, y, z)
	{}

	inline TQuaternion(double roll, double pitch, double yaw)
	{
		Quaterniond q = fromRPY(roll, pitch, yaw);
		coeffs() = q.coeffs();
	}

	inline TQuaternion(const Quaterniond &q)
	{ coeffs() = q.coeffs(); }

	inline TQuaternion(const Eigen::Matrix3d &R)
	{
		Quaterniond Q(R);
		coeffs() = Q.coeffs();
	}

	inline TQuaternion& operator=(const Quaterniond &q)
	{
		coeffs() = q.coeffs();
		return *this;
	}

	inline double roll() const
	{ return quaternionToRPY(*this).x(); }

	inline double pitch() const
	{ return quaternionToRPY(*this).y(); }

	inline double yaw() const
	{ return quaternionToRPY(*this).z(); }

	TQuaternion operator * (const double &mul) const;
	TQuaternion operator / (const double &div) const;
};


struct TTransform : public Eigen::Affine3d
{
	TTransform()
	{ m_matrix = Eigen::Matrix4d::Identity(); }

	TTransform(const Eigen::Affine3d &a)
	{ m_matrix = a.matrix(); }

	template <typename Derived>
	TTransform(const Eigen::MatrixBase<Derived> &M)
	{
		assert(M.cols()==4 and M.rows()==4);
		// XXX: Check singularity of rotation part
		for(int j=0; j<4; ++j) {
			for (int i=0; i<4; ++i) {
				m_matrix(i,j) = M(i,j);
		}}
	}

	TTransform(
		const double X, const double Y, const double Z,
		const double QX, const double QY, const double QZ, const double QW);

	TTransform(
		const double X, const double Y, const double Z,
		const double roll, const double pitch, const double yaw);

	static TTransform from_XYZ_RPY
		(const Eigen::Vector3d &pos,
		double roll=0, double pitch=0, double yaw=0);

	static TTransform from_Pos_Quat
		(const Eigen::Vector3d &pos,
			const Eigen::Quaterniond &ori
				=Eigen::Quaterniond::Identity());

	static TTransform from_R_t
		(const Eigen::Vector3d &t, const Eigen::Matrix3d &R);

	inline const Vector3d position() const
	{ return this->translation(); }

	inline const TQuaternion orientation() const
	{ return TQuaternion(this->rotation()); }

	std::string
	str (bool simple=false) const;

	void
	displacement (const TTransform &other, double &linear, double &angular) const;

	static TTransform interpolate(const TTransform &T1, const TTransform &T2, const double ratio);

	template<class Archive>
	inline void save(Archive &ar, const unsigned int v) const
	{
		ar << boost::serialization::make_array(data(), 16);
	}

	template<class Archive>
	inline void load(Archive &ar, const unsigned int v)
	{
		ar >> boost::serialization::make_array(data(), 16);
	}

	const double x() const
	{ return position().x(); }

	const double y() const
	{ return position().y(); }

	const double z() const
	{ return position().z(); }

	const double qx() const
	{ return orientation().x(); }

	const double qy() const
	{ return orientation().y(); }

	const double qz() const
	{ return orientation().z(); }

	const double qw() const
	{ return orientation().w(); }

	bool isValid() const;

	TTransform shift(const Eigen::Vector3d &vs) const;
	inline TTransform shift(const double &x, const double &y, const double &z) const
	{ return shift(Eigen::Vector3d(x, y, z)); }

	TTransform rotate(const double roll, const double pitch=0, const double yaw=0) const;

	// Used for `transformation per time unit' and multiplication of velocity against time
	TTransform operator * (const double &div) const;
	TTransform operator / (const double &div) const;

	inline const TTransform operator * (const TTransform& other) const
	{ return Eigen::Affine3d::operator*(other); }

	inline const Eigen::Vector3d operator * (const Eigen::Vector3d &V) const
	{ return Eigen::Affine3d::operator*(V); }

/*
	TTransform operator / (const tduration &td) const
	{ return *this / toSeconds(td); }
*/

	BOOST_SERIALIZATION_SPLIT_MEMBER()
};


typedef TTransform Pose;


/*
 * Calculate normalized cumulative distribution function over a histogram of an image
 */
Eigen::VectorXd cdf (const cv::Mat &grayImage, const cv::Mat &mask=cv::Mat());


/*
 * Debugging vectors and quaternions by output to string
 */
std::string dumpVector(const Eigen::Vector3d &v);

std::string dumpVector(const Eigen::Quaterniond &v);

std::string dumpVector(const TTransform &P);


inline ptime getCurrentTime ()
{ return boost::posix_time::microsec_clock::local_time(); }

inline double td_seconds (const tduration &td)
{ return (double(td.total_microseconds()) / 1e6); }

double toSeconds (const ptime &pt);

inline double toSeconds (const tduration &td)
{ return (double(td.total_microseconds()) / 1e6); }

/*
 * Convert UNIX timestamp in seconds to ptime
 */
ptime fromSeconds (const double s);

void debugMsg(const std::string &s, double is_error=false);


#define RecordRuntime(funcDef, CALL)  \
	ptime _t1_ = getCurrentTime(); \
	CALL ; \
	ptime _t2_ = getCurrentTime(); \
	tduration _td_ = _t2_ - _t1_ ; \
	debugMsg(string(funcDef) + " (seconds): " + to_string(double(_td_.total_microseconds()) / 1e6)); \


inline boost::filesystem::path getMyPath()
{ return boost::filesystem::path(ros::package::getPath("vmml")); }


/*
 * Our own pseudo-inverse routine, in case Eigen does not provide it
 */
template<typename Scalar, int r, int c>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
pseudoInverse(const Eigen::Matrix<Scalar,r,c> &M, const Scalar cutoff=1e-10)
{
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> pinv;

	Eigen::JacobiSVD <Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> svd(M, Eigen::ComputeFullU|Eigen::ComputeFullV);
	auto U = svd.matrixU();
	auto V = svd.matrixV();
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Sx = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(M.cols(), M.rows());
	for (auto i=0; i<M.cols(); ++i) {
		Scalar s = svd.singularValues()[i];
		if (fabs(s)<=cutoff)
			Sx(i,i) = 0;
		else Sx(i,i) = 1/s;
	}
	pinv = V * Sx * U.transpose();

	return pinv;
}


#endif /* UTILITIES_H_ */
