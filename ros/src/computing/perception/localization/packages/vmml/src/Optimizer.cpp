/*
 * optimizer.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: sujiwo
 */

#include <memory>
#include <random>
#include <algorithm>

#include "VMap.h"
#include "Optimizer.h"
#include "BaseFrame.h"
#include "Frame.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/factory.h"
//#include "g2o/types/sim3/types_seven_dof_expmap.h"


/*
 * ROS Melodic/Kinetic handler
 */
#include <ros/ros.h>
#include <ros/common.h>


#if ROS_VERSION_MINIMUM(1,14,0)

template<typename LinearSolverT, typename BlockSolverT, typename OptimizationAlgorithmT>
OptimizationAlgorithmT*
createSolverAlgorithm()
{
	typename BlockSolverT::LinearSolverType* ls = new LinearSolverT();
	auto lsptr = std::unique_ptr<typename BlockSolverT::LinearSolverType>(ls);
	auto solver = std::unique_ptr<g2o::Solver>(new BlockSolverT(std::move(lsptr)));
	return new OptimizationAlgorithmT(std::move(solver));
}


#else
#if ROS_VERSION_MINIMUM(1,12,0)

template<typename LinearSolverT, typename BlockSolverT, typename OptimizationAlgorithmT>
OptimizationAlgorithmT*
createSolverAlgorithm()
{
	auto ls = new LinearSolverT;
	auto solver = new BlockSolverT(ls);
	return new OptimizationAlgorithmT(solver);
}

#else
#error "Unsupported G2O version"
#endif
#endif


using namespace std;
using namespace Eigen;


typedef uint64 oid;

const float thHuber2D = sqrt(5.99);
const float thHuber3D = sqrt(7.815);

const int baIteration = 6;


class PoseDisturb
{
public:
	typedef std::uniform_real_distribution<double> UDistT;
	typedef std::shared_ptr<UDistT> UDistPtr;


	PoseDisturb(const Vector3d &means):
		meanShift(means),
		distributions(vector<UDistPtr>(3, nullptr))
	{
		if (meanShift.x()!=0.0) {
			distributions[0] = createUniformDistribution(meanShift.x());
		}
		if (meanShift.y()!=0.0) {
			distributions[1] = createUniformDistribution(meanShift.y());
		}
		if (meanShift.z()!=0.0) {
			distributions[2] = createUniformDistribution(meanShift.z());
		}
	}

	Pose disturb(const Pose &p)
	{
		Vector3d sh = getRandomShift();
		return p.shift(sh);
	}

	static UDistPtr createUniformDistribution(const double &m)
	{
		return UDistPtr(new UDistT(-m, m));
	}

protected:
	Vector3d meanShift;
	default_random_engine generator;
	vector<UDistPtr> distributions;

	Vector3d getRandomShift()
	{
		Vector3d sh = Vector3d::Zero();
		for (int i=0; i<3; ++i)
			if (distributions[i]!=nullptr) {
				UDistT &dist = *distributions[i];
				sh[i] = dist(generator);
			}

		return sh;
	}
};


void VertexCameraMono::set(KeyFrame *_f)
{
	kf = _f;
	setEstimate(kf->toSE3Quat());
}


void VertexMapPoint::set(MapPoint *_p)
{
	mp = _p;
	setEstimate(mp->getPosition());
}


void EdgeProjectMonocular::set(VertexCameraMono *frame, VertexMapPoint *point)
{
	const VMap &myMap = frame->kf->getParent();
	Vector2d obs = frame->kf->keypointv(point->getKeyPointId(frame));
	setMeasurement(obs);
	setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(point));
	setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(frame));

	auto mKeypoint = frame->kf->keypoint(point->getKeyPointId(frame));
	Matrix2d edgeInfo = Matrix2d::Identity() * 1.2 * (mKeypoint.octave+1);
	setInformation(Matrix2d::Identity() * 1.2 * (mKeypoint.octave+1));
}


Vector3d EdgeProjectMonocular::transformWorldPointToFrame
(const Vector3d &pointInWorld)
const
{
	const auto *vKf = static_cast<const VertexCameraMono*>(vertex(1));
	auto est = vKf->estimate();
	return est.map(pointInWorld);
}


bool EdgeProjectMonocular::isDepthPositive() const
{
	const g2o::VertexSBAPointXYZ &point = *static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
	Vector3d ptByCam = transformWorldPointToFrame(point.estimate());
	return (ptByCam.z() >= 0.0);
}


G2O_REGISTER_TYPE(VERTEX_CAMERA_MONO, VertexCameraMono);
G2O_REGISTER_TYPE(VERTEX_MAP_POINT, VertexMapPoint);
G2O_REGISTER_TYPE(EDGE_PROJECT_MONOCULAR, EdgeProjectMonocular);



g2o::SE3Quat toSE3Quat (const KeyFrame &kf)
{
	Matrix4d extMat = kf.externalParamMatrix4();
	return g2o::SE3Quat(extMat.block<3,3>(0,0), extMat.block<3,1>(0,3));
}


g2o::SE3Quat toSE3Quat (const BaseFrame &frame)
{
	Matrix4d extMat = frame.externalParamMatrix4();
	return g2o::SE3Quat(extMat.block<3,3>(0,0), extMat.block<3,1>(0,3));
}


g2o::SE3Quat toSE3Quat (const Pose &spose)
{
	Matrix4d extMat = BaseFrame::createExternalParamMatrix4(spose);
	return g2o::SE3Quat(extMat.block<3,3>(0,0), extMat.block<3,1>(0,3));
}


void fromSE3Quat (const g2o::SE3Quat &pose, KeyFrame &kf)
{
	kf.orientation() = pose.rotation().inverse();
	kf.position() = -(kf.orientation() * pose.translation());
}


void fromSE3Quat (const g2o::SE3Quat &pose, BaseFrame &frb)
{
	auto Q = pose.rotation().inverse();
	auto P = -(Q * pose.translation());
	frb.setPose(P, Q);
}


void fromSE3Quat (const g2o::SE3Quat &pose, Pose &target)
{
	auto Q = pose.rotation().inverse();
	auto P = -(Q * pose.translation());
	target = Pose::from_Pos_Quat(P, Q);
}


#include <cstdio>
void debugVector (const VectorXd &v)
{
	if (v.size()==2)
		printf ("(%f, %f)\n", v.x(), v.y());
	else if (v.size()==3)
		printf ("(%f, %f, %f)\n", v.x(), v.y(), v.z());
}


void bundle_adjustment (VMap *orgMap)
{
	vector<kfid> keyframeList = orgMap->allKeyFrames();
	vector<mpid> mappointList = orgMap->allMapPoints();

	g2o::SparseOptimizer optimizer;
	optimizer.setVerbose(true);
/*
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
*/
	auto solver = createSolverAlgorithm<
			g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>,
			g2o::BlockSolver_6_3,
			g2o::OptimizationAlgorithmLevenberg>();
	optimizer.setAlgorithm(solver);

	// XXX: This routine does not support multiple camera
	const CameraPinholeParams cp = orgMap->getCameraParameter(0);
	g2o::CameraParameters *camParams =
		new g2o::CameraParameters(cp.fx, Vector2d(cp.cx,cp.cy), 0);
	camParams->setId(0);
	optimizer.addParameter(camParams);

	map<oid, kfid> vertexKfMap;
	map<kfid, g2o::VertexSE3Expmap*> vertexKfMapInv;
	map<oid, mpid> vertexMpMap;
	map<mpid, g2o::VertexSBAPointXYZ*> vertexMpMapInv;
	oid vId = 1;

	for (kfid &kId: keyframeList) {

		g2o::VertexSE3Expmap *vKf = new g2o::VertexSE3Expmap();
		KeyFrame *kf = orgMap->keyframe(kId);
		vKf->setEstimate (toSE3Quat(*kf));
		vKf->setId(vId);
		vKf->setFixed(kId<2);
		optimizer.addVertex(vKf);
		vertexKfMap.insert(pair<oid, kfid> (vId, kId));
		vertexKfMapInv[kId] = vKf;
		vId ++;

		for (auto &ptr: orgMap->allMapPointsAtKeyFrame(kId)) {

			const MapPoint *mp = orgMap->mappoint(ptr.first);
			const cv::KeyPoint p2K = kf->keypoint(ptr.second);

			g2o::VertexSBAPointXYZ *vMp = new g2o::VertexSBAPointXYZ();

			// What effect if we run this line ?
			vMp->setFixed(false);

			vMp->setEstimate(mp->getPosition());
			vMp->setMarginalized(true);
			vMp->setId(vId);
			optimizer.addVertex(vMp);
			vertexMpMap.insert(pair<oid,mpid> (vId, mp->getId()));
			vertexMpMapInv[mp->getId()] = vMp;
			vId++;

			// Edges
			g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
			edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vMp));
			edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vKf));
			edge->setMeasurement(Vector2d(p2K.pt.x, p2K.pt.y));
			Matrix2d uncertainty = Matrix2d::Identity() * (1.2*(p2K.octave+1));
			edge->setInformation(uncertainty);
			edge->setParameterId(0, 0);

			// XXX: Should we add a robust kernel for the edge here, ie. like pose optimization below ?

			optimizer.addEdge(edge);
		}
	}

	optimizer.initializeOptimization();
	// XXX: Determine number of iterations
	optimizer.optimize(baIteration);

	// Recovery of optimized data
	// KeyFrames
	for (auto &kVpt: vertexKfMap) {

		oid vId = kVpt.first;
		kfid kId = kVpt.second;
		KeyFrame *kf = orgMap->keyframe(kId);
		g2o::VertexSE3Expmap *vKfSE3 = static_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(vId));

		g2o::SE3Quat kfPoseSE3 = vKfSE3->estimate();
		fromSE3Quat(kfPoseSE3, *kf);
	}

	// MapPoints
	for (auto &mVpt: vertexMpMap) {
		oid vId = mVpt.first;
		mpid mid = mVpt.second;
		MapPoint *mp = orgMap->mappoint(mid);
		g2o::VertexSBAPointXYZ *vMp = static_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(vId));
		mp->setPosition(vMp->estimate());
	}
}


void bundle_adjustment_2 (VMap *orgMap)
{
	vector<kfid> keyframeList = orgMap->allKeyFrames();
	vector<mpid> mappointList = orgMap->allMapPoints();

	g2o::SparseOptimizer optimizer;
	optimizer.setVerbose(true);
/*
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
*/
	auto solver = createSolverAlgorithm<
			g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>,
			g2o::BlockSolver_6_3,
			g2o::OptimizationAlgorithmLevenberg >();
	optimizer.setAlgorithm(solver);

	// XXX: This routine does not support multiple camera
	const CameraPinholeParams cp = orgMap->getCameraParameter(0);
	g2o::CameraParameters *camParams =
		new g2o::CameraParameters(cp.fx, Vector2d(cp.cx,cp.cy), 0);
	camParams->setId(0);
	optimizer.addParameter(camParams);

	map<oid, kfid> vertexKfMap;
	map<kfid, g2o::VertexCam*> vertexKfMapInv;
	map<oid, mpid> vertexMpMap;
	map<mpid, g2o::VertexSBAPointXYZ*> vertexMpMapInv;
	oid vId = 1;

	/*
	 * Camera's Pose Disturbance are specified in lateral, longitudinal and vertical ways
	 * For lateral, specify in X
	 * For longitudinal, specify in Z
	 * For vertical, specify in Y
	 */
	//                            X    Y    Z
	PoseDisturb poseDist(Vector3d(3,   0.1,   0));

	// Stage #1: Build vertices and edges from-KF-to-MP
	for (kfid &kId: keyframeList) {

		g2o::VertexCam *vKf = new g2o::VertexCam();
		KeyFrame *kf = orgMap->keyframe(kId);

		Pose nPose = kf->pose();
		nPose = poseDist.disturb(nPose);
		kf->setPose(nPose);
		vKf->setEstimate (kf->forG2O());

		vKf->setId(vId);
		vKf->setFixed(kId<2);
		optimizer.addVertex(vKf);
		vertexKfMap.insert(pair<oid, kfid> (vId, kId));
		vertexKfMapInv[kId] = vKf;
		vId ++;

		for (auto &ptr: orgMap->allMapPointsAtKeyFrame(kId)) {

			const MapPoint *mp = orgMap->mappoint(ptr.first);
			const cv::KeyPoint p2K = kf->keypoint(ptr.second);

			g2o::VertexSBAPointXYZ *vMp = new g2o::VertexSBAPointXYZ();

			// What effect if we run this line ?
			vMp->setFixed(false);

			vMp->setEstimate(mp->getPosition());
			vMp->setMarginalized(true);
			vMp->setId(vId);
			optimizer.addVertex(vMp);
			vertexMpMap.insert(pair<oid,mpid> (vId, mp->getId()));
			vertexMpMapInv[mp->getId()] = vMp;
			vId++;

			// Edges
			g2o::EdgeProjectP2MC *edge = new g2o::EdgeProjectP2MC();
			edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vMp));
			edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vKf));
			edge->setMeasurement(Vector2d(p2K.pt.x, p2K.pt.y));
			Matrix2d uncertainty = Matrix2d::Identity() * (1.2*(p2K.octave+1));
			edge->setInformation(uncertainty);
//			edge->setParameterId(0, 0);

			// XXX: Should we add a robust kernel for the edge here, ie. like pose optimization below ?

			optimizer.addEdge(edge);
		}
	}

	// Stage #2: Build edges for inter-keyframe edges
	for (kfid &kId: keyframeList) {

		vector<kfid> kfFriends = orgMap->getKeyFramesComeInto(kId);
		if (kfFriends.empty()==true)
			continue;

		for (auto &kfriend: kfFriends) {
			g2o::EdgeSBACam *edgeKf = new g2o::EdgeSBACam;
			edgeKf->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>( vertexKfMapInv[kfriend] ));
			edgeKf->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( vertexKfMapInv[kId] ));

			// Measurement
			// P2 = P1 * T ==> T = inverse(P1) * P2
			TTransform T = orgMap->keyframe(kfriend)->pose().inverse() * orgMap->keyframe(kId)->pose();
			edgeKf->setMeasurement(g2o::SE3Quat(T.orientation(), T.position()));

			// Uncertainty
			// XXX: Unfinished
			Matrix<double,6,6> edgeInf = Matrix<double,6,6>::Identity();
			// For GNSS
			edgeInf(0,0) = 5.0;
			edgeInf(1,1) = 1.0;
			edgeInf(2,2) = 5.0;
			edgeKf->setInformation(edgeInf);

			optimizer.addEdge(edgeKf);
		}

	}

	optimizer.initializeOptimization();
	// XXX: Determine number of iterations
	optimizer.optimize(baIteration);

	// Recovery of optimized data
	// KeyFrames
	for (auto &kVpt: vertexKfMap) {

		oid vId = kVpt.first;
		kfid kId = kVpt.second;
		KeyFrame *kf = orgMap->keyframe(kId);
		g2o::VertexCam *vKfSE3 = static_cast<g2o::VertexCam*> (optimizer.vertex(vId));

		g2o::SE3Quat kfPoseSE3 = vKfSE3->estimate();
		fromSE3Quat(kfPoseSE3, *kf);
	}

	// MapPoints
	for (auto &mVpt: vertexMpMap) {
		oid vId = mVpt.first;
		mpid mid = mVpt.second;
		MapPoint *mp = orgMap->mappoint(mid);
		g2o::VertexSBAPointXYZ *vMp = static_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(vId));
		mp->setPosition(vMp->estimate());
	}
}



/*
 * We would rather not store the computed pose into Frame, as the decision to use the pose (or
 * not) is left for the caller
 */
int optimize_pose (const Frame &frame, Pose &initPose, const VMap *vmap)
{
	g2o::SparseOptimizer optimizer;
/*
	g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
	linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
*/
	auto solver = createSolverAlgorithm<
			g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>,
			g2o::BlockSolver_6_3,
			g2o::OptimizationAlgorithmLevenberg >();
	optimizer.setAlgorithm(solver);

	int nInitialCorrespondences=0;

	const CameraPinholeParams cp = frame.getCameraParameters();
	g2o::CameraParameters *camParams =
		new g2o::CameraParameters((cp.fx+cp.fy)/2, Vector2d(cp.cx,cp.cy), 0);
	camParams->setId(0);
	optimizer.addParameter(camParams);

	// Set Frame vertex
	g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
	vSE3->setEstimate(toSE3Quat(initPose));
	vSE3->setId(1);
	vSE3->setFixed(false);
	optimizer.addVertex(vSE3);

	// Set MapPoint vertices
	const auto &mapProjections = frame.getVisibleMapPoints();
	const int N = mapProjections.size();

	vector<g2o::EdgeProjectXYZ2UV*> vpEdgesMono;
	vector<size_t> vnIndexEdgeMono;
	vpEdgesMono.reserve(N);
	vnIndexEdgeMono.reserve(N);
	vector<bool> isMpOutliers (N, false);

	const float deltaMono = sqrt(5.991);
	const float deltaMono2 = 5.991;

	int i=0, vId;
	for (auto &vpmp: mapProjections) {
		const MapPoint &mp = *vmap->mappoint(vpmp.first);

		const cv::KeyPoint &kp = frame.keypoint(vpmp.second);
		Vector2d obs (kp.pt.x, kp.pt.y);

		g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV;
		edge->setMeasurement(obs);

		// It still amazes me how 2D uncertainty is measured ...
		Matrix2d uncertainty = Matrix2d::Identity() * (1.2*(kp.octave+1));
		edge->setInformation(uncertainty);
		edge->setParameterId(0, 0);

		g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
		edge->setRobustKernel(rk);
		rk->setDelta(deltaMono);

		// Put MP's world coordinate, as vertex
		g2o::VertexSBAPointXYZ *vMp = new g2o::VertexSBAPointXYZ;
		vId = i+2;
		vMp->setId(vId);
		vMp->setFixed(true);
		vMp->setMarginalized(true);
		vMp->setEstimate(mp.getPosition());
		optimizer.addVertex(vMp);

		edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(vId)));
		edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(1)));
		optimizer.addEdge(edge);

		vpEdgesMono.push_back(edge);
		vnIndexEdgeMono.push_back(i);

		++i;
	}

	// Run optimization 4 times
	// Credit to original ORB-SLAM
	const int numOfOptimizationIteration = 10;
	int numOutliers = 0;

	for (int it=0; it<4; ++it) {
		vSE3->setEstimate(toSE3Quat(initPose));
		optimizer.initializeOptimization(0);
		optimizer.optimize(numOfOptimizationIteration);

		numOutliers = 0;
		for (int ix=0; ix<vpEdgesMono.size(); ++ix) {

			auto edge = vpEdgesMono[ix];
			auto idx = vnIndexEdgeMono[ix];

			if (isMpOutliers[ix]) {
				edge->computeError();
			}

			const double xi2 = edge->chi2();
			if (xi2 > deltaMono2) {
				isMpOutliers[ix] = true;
				edge->setLevel(1);
				numOutliers += 1;
			}

			else {
				isMpOutliers[ix] = false;
				edge->setLevel(0);
			}

			if (it==2)
				edge->setRobustKernel(0);
		}

		if (optimizer.edges().size() < 10)
			break;
	}

	// Recover pose
	g2o::VertexSE3Expmap* vSE3_new = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(1));
	g2o::SE3Quat SE3quat_new = vSE3_new->estimate();
	fromSE3Quat(SE3quat_new, initPose);

	return N-numOutliers;
}


void local_bundle_adjustment (VMap *origMap, const kfid &targetKf)
{
	// Find connected keyframes from targetKf
	vector<kfid> neighbourKfs = origMap->getKeyFramesComeInto(targetKf);
	std::sort(neighbourKfs.begin(), neighbourKfs.end());
	neighbourKfs.push_back(targetKf);

	// Local MapPoints seen in Local KeyFrames
	set<mpid> relatedMps;
	for (auto &kfl: neighbourKfs) {
		for (auto &mpair: origMap->allMapPointsAtKeyFrame(kfl)) {
			relatedMps.insert(mpair.first);
		}
	}

	// Fixed KeyFrames: those that see local MapPoints but not included in connected keyframes
	set<kfid> fixedKfs;
	for (auto &mp: relatedMps) {
		auto curRelatedKf = origMap->getRelatedKeyFrames(mp);
		for (auto &kf: curRelatedKf) {
			if (std::find(neighbourKfs.begin(), neighbourKfs.end(), kf) != neighbourKfs.end())
				continue;
			fixedKfs.insert(kf);
		}
	}

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
/*
	auto *linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>;
	auto solverPtr = new g2o::BlockSolver_6_3(linearSolver);
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solverPtr);
*/
	auto solver = createSolverAlgorithm<
			g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>,
			g2o::BlockSolver_6_3,
			g2o::OptimizationAlgorithmLevenberg >();
	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(true);

	map<int, kfid> vertexKfMap;
	map<kfid, int> vertexKfMapInv;
//	map<int, mpid> vertexMpMap;
	map<mpid, int> vertexMpMapInv;
	int vId = 1;

	// Local KeyFrame vertices
//	vector<g2o::VertexCam> localKfVertices(neighbourKfs.size());
	int i = 0;
	for (auto &kf: neighbourKfs) {
		auto Kf = origMap->keyframe(kf);
		auto localKfVertex = new VertexCameraMono;
		localKfVertex->set(Kf);
		localKfVertex->setId(vId);
		localKfVertex->setFixed(vId==1);
		optimizer.addVertex(localKfVertex);
		vertexKfMap[vId] = kf;
		vertexKfMapInv[kf] = vId;
		++i;
		vId++;
	}

	// Fixed keyframe vertices
//	vector<g2o::VertexCam> fixedKfVertices(fixedKfs.size());
	i = 0;
	for (auto &kf: fixedKfs) {
		auto Kf = origMap->keyframe(kf);
		auto fixedKfVertex = new VertexCameraMono;
		fixedKfVertex->set(Kf);
		fixedKfVertex->setId(vId);
		fixedKfVertex->setFixed(true);
		optimizer.addVertex(fixedKfVertex);
		vertexKfMap[vId] = kf;
		vertexKfMapInv[kf] = vId;
		++i;
		vId++;
	}

	// How many edges do we need ?
	int numEdges = 0;
	for (auto &mp: relatedMps)
		numEdges += origMap->countRelatedKeyFrames(mp);

	// Camera Parameter
	auto Camera0 = origMap->getCameraParameter(0);
	auto* myCamera = new g2o::CameraParameters(Camera0.f(), Camera0.principalPoints(), 0);
	myCamera->setId(0);
	optimizer.addParameter(myCamera);

	const double thHuberDelta = sqrt(5.991);

	// MapPoint Vertices
//	vector<g2o::VertexSBAPointXYZ> relatedMpVertices(relatedMps.size());
	vector<EdgeProjectMonocular*> edgesMpKf(numEdges);
	i = 0;
	int j = 0;
	for (auto &mp: relatedMps) {
		auto *vertexMp = new VertexMapPoint;
		auto Mp = origMap->mappoint(mp);
		vertexMp->setFixed(false);
		vertexMp->set(Mp);
		vertexMp->setId(vId);
		vertexMp->setMarginalized(true);
		optimizer.addVertex(vertexMp);

//		vertexMpMap[vId] = mp;
		vertexMpMapInv[mp] = vId;

		// Create Edges
		for (auto &kf: origMap->getRelatedKeyFrames(mp)) {

			auto *edge = new EdgeProjectMonocular;
			edge->setParameterId(0, 0);
			edgesMpKf[j] = edge;

			auto vKf = dynamic_cast<VertexCameraMono*>(optimizer.vertex(vertexKfMapInv.at(kf)));
			edge->set(vKf, vertexMp);

			// Debugging purpose
			Vector3d
				transvec1 = edge->transformWorldPointToFrame(Mp->getPosition()),
				transvec2 = origMap->keyframe(kf)->transform(Mp->getPosition());

			auto *robustKernel = new g2o::RobustKernelHuber;
			edge->setRobustKernel(robustKernel);
			robustKernel->setDelta(thHuberDelta);

			optimizer.addEdge(edge);
			j++;
		}

		++i;
		vId++;
	}

	optimizer.initializeOptimization();
	optimizer.optimize(5);

	// Check inliers
	for (auto edge: edgesMpKf) {
		double c = edge->chi2();
		auto dp = edge->isDepthPositive();
		if (c > 5.991 or !dp) {
			edge->setLevel(1);
		}

		edge->setRobustKernel(nullptr);
	}

	// Re-optimize without outliers
	optimizer.initializeOptimization(0);
	optimizer.optimize(10);

	// Recover optimized data
	for (auto &kfl: neighbourKfs) {
		auto kfVtxId = vertexKfMapInv.at(kfl);
		auto vKf = static_cast<VertexCameraMono*> (optimizer.vertex(kfVtxId));
		vKf->updateToMap();
	}

	for (auto &mp: relatedMps) {
		auto mpVtxId = vertexMpMapInv.at(mp);
		auto vPoint = static_cast<VertexMapPoint*> (optimizer.vertex(mpVtxId));
		vPoint->updateToMap();
	}

	cout << "Local BA Done\n";
}
