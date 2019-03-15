/*
 * MapBuilder2.cpp
 *
 *  Created on: Jul 26, 2018
 *      Author: sujiwo
 */

#include <exception>
#include <thread>
#include <list>

#include "Matcher.h"
#include "MapBuilder2.h"
#include "Optimizer.h"
#include "ImageDatabase.h"
#include "Viewer.h"
#include "Tracklet.h"
#include "utilities.h"




using namespace std;
using namespace Eigen;


MapBuilder2::frameCallback defaultFrameCallback =
[&] (const InputFrame &f)
{};


MapBuilder2::MapBuilder2() :
	kfAnchor(0),
	inputCallback(defaultFrameCallback)
{
	cMap = new VMap();
//	imageView = new Viewer;
}


MapBuilder2::~MapBuilder2()
{
//	delete(imageView);
}


void
MapBuilder2::initialize (const InputFrame &f1, const InputFrame &f2)
{
	cout << "Comparing for init: " << f1.sourceId << "->" << f2.sourceId << endl;
	kfid k1 = cMap->createKeyFrame(f1.image, f1.position, f1.orientation, f1.cameraId, NULL, f1.sourceId, f1.tm);
	kfid k2 = cMap->createKeyFrame(f2.image, f2.position, f2.orientation, f2.cameraId, NULL, f2.sourceId, f2.tm);

	double translationHint = -1.0;
/*
	if (datasetType==MeidaiType) {
		MeidaiBagDataset::Ptr meidaiDs = static_pointer_cast<MeidaiBagDataset>(sourceDataset);
		MeidaiDataItem::Ptr
			f1Di = meidaiDs->getNative(f1.sourceId),
			f2Di = meidaiDs->getNative(f2.sourceId);
		TTransform T12 = Matcher::matchLidarScans(*f1Di, *f2Di);
		translationHint = T12.translation().norm();
	}
	else {
		translationHint = (f1.position - f2.position).norm();
	}
*/
	// Cheating !
	translationHint = (f1.position - f2.position).norm();

	cMap->estimateStructure(k1, k2, translationHint);
	kfAnchor = k2;
	ifrAnchor = f2;

	localBAAnchor = k1;

	auto myneigh = cMap->getKeyFramesComeInto(kfAnchor);
	return;
}


void
MapBuilder2::track (const InputFrame &f)
{
	if (initialized==false)
		throw runtime_error("Map not initialized");

	// XXX: Get correct metric scale of the two frames
	double translationHint = 1.0;
/*
	if (datasetType==MeidaiType) {
		MeidaiBagDataset::Ptr meidaiDs = static_pointer_cast<MeidaiBagDataset>(sourceDataset);
		MeidaiDataItem::Ptr
			anchorDi = meidaiDs->getNative(ifrAnchor.sourceId),
			curDi = meidaiDs->getNative(f.sourceId);
		TTransform T12 = Matcher::matchLidarScans(*anchorDi, *curDi);
		dist2Frame = T12.translation().norm();
	}
*/
	// Cheating !
	translationHint = (f.position - ifrAnchor.position).norm();

	kfid fId = cMap->createKeyFrame(f.image, f.position, f.orientation, f.cameraId, NULL, f.sourceId, f.tm);

	cout << "Comparing for tracking: " << ifrAnchor.sourceId << "->" << f.sourceId << endl;

	cMap->estimateAndTrack(kfAnchor, fId, translationHint);

	cMap->keyframe(fId)->previousKeyframe = kfAnchor;

	// XXX: Decide when to move the anchor
	kfAnchor = fId;
	ifrAnchor = f;

	auto myneigh = cMap->getKeyFramesComeInto(kfAnchor);
	return;
}


void
MapBuilder2::input(const InputFrame &f)
{
	if (isNormalFrame(f)==false)
		return;

	double runTrans, runRot;

	if (initialized==false) {

		if (frame0.image.empty()) {
			frame0 = f;
			cerr << "Input#: " << f.sourceId << endl;
			return;
		}

		else {
			f.getPose().displacement(frame0.getPose(), runTrans, runRot);
			if (runTrans>=translationThrsInit or runRot>=rotationThrs) {
				initialize(frame0, f);
				inputCallback(f);
				initialized = true;
				cerr << "Initialized; # of map points: " << cMap->allMapPoints().size() << endl;
				cerr << "Input#: " << f.sourceId << endl;
				return;
			}
		}
	}

	else {
		ifrAnchor.getPose().displacement(f.getPose(), runTrans, runRot);
		if (runTrans>=translationThrs or runRot>=rotationThrs) {
			kfid lastAnchor = kfAnchor;
			track (f);

			cerr << "Input#: " << f.sourceId << endl;

			// Build connections
			vector<kfid> kfInsToAnchor = cMap->getKeyFramesComeInto(lastAnchor);
			cerr << "Found " << kfInsToAnchor.size() << " input keyframes\n";
			const kfid targetKfId = kfAnchor;

			// XXX: Parallelize this
			for (auto &kfx: kfInsToAnchor) {
				cMap->trackMapPoints(kfx, targetKfId);
			}

			// Check whether we need local BA
			kfInsToAnchor = cMap->getKeyFramesComeInto(kfAnchor);
			if (std::find(kfInsToAnchor.begin(), kfInsToAnchor.end(), localBAAnchor)==kfInsToAnchor.end()) {
				cout << "Local BA running\n";
				auto vAnchors = cMap->getKeyFramesComeInto(lastAnchor);
				local_bundle_adjustment(cMap, lastAnchor);
				localBAAnchor = lastAnchor;
			}

			inputCallback(f);
		}
	}
}


/*
 * This function decides when a frame is `good enough' in terms of exposure
 * to be included for map building
 */
bool
MapBuilder2::isNormalFrame (const InputFrame &f)
{
	// throw away over-exposed frames
	// XXX: also need to do the same for under-exposed frame
	auto normcdf = cdf(f.image);
	if (normcdf[127] < 0.25)
		return false;
	else return true;
}


void
MapBuilder2::build ()
{
	mapPointCulling();
	cMap->fixFramePointsInv();

	thread ba([this] {
				cout << "Bundling...";

// Currently, global bundle adjustment is disabled so that map can be finished faster.
// Global BA may be run from vmml_cli separately after loading the map

//				bundle_adjustment(cMap);
//				bundle_adjustment_2(cMap);
				cout << "BA Done\n";
	});

	thread db([this] {
				cout << "Rebuilding Image DB... ";
				cout.flush();
				cMap->getImageDB()->rebuildAll();
				cout << "Image DB Build Done\n";
	});

	ba.join();
	db.join();
	return;
}


InputFrame createInputFrameX (GenericDataItem::ConstPtr DI)
{
	// We prefer gray images
	cv::Mat img=DI->getImage();
	cv::cvtColor(img, img, CV_BGR2GRAY, 1);

	InputFrame f(
		img,
		DI->getPosition(),
		DI->getOrientation(),
		DI->getId()
	);
	f.tm = DI->getTimestamp();

	return f;

}


void
MapBuilder2::runFromDataset(GenericDataset::Ptr sourceDs, const ptime startTime, const ptime stopTime)
{
	if (initialized != false)
		throw runtime_error("Map process has been running; aborted");

	if (startTime < sourceDs->first()->getTimestamp()
		or stopTime > sourceDs->last()->getTimestamp())
		throw runtime_error("Requested times are outside of dataset range");

	dataItemId
		startId = sourceDs->getLowerBound(startTime),
		stopId = sourceDs->getLowerBound(stopTime);

	return runFromDataset(sourceDs, startId, stopId);
}


void
MapBuilder2::runFromDataset(GenericDataset::Ptr sourceDs, dataItemId startPos, dataItemId stopPos)
{
	sourceDataset = sourceDs;
	cMap->reset();

	if (startPos==std::numeric_limits<dataItemId>::max() and
		stopPos==std::numeric_limits<dataItemId>::max()) {
		startPos = 0;
		stopPos = sourceDataset->size()-1;
	}

	for (auto currentId=startPos; currentId<=stopPos; ++currentId) {
		InputFrame cFrame = createInputFrameX (sourceDataset->get(currentId));
		this->input(cFrame);
	}

	this->build();
}


void
MapBuilder2::runFromDataset
(MeidaiBagDataset::Ptr sourceDs,
	dataItemId startPos,
	dataItemId stopPos)
{
	datasetType = MeidaiType;
	return runFromDataset(static_pointer_cast<GenericDataset>(sourceDs), startPos, stopPos);
}


bool
MapBuilder2::checkDataPoints (GenericDataset::ConstPtr sourceDs, const ptime startTime, const ptime stopTime)
const
{
	dataItemId
		startId = sourceDs->getLowerBound(startTime),
		stopId = sourceDs->getLowerBound(stopTime);

	for (auto currentId=startId; currentId<=stopId; ++currentId) {
		auto curFrame = sourceDs->get(currentId);
		if (curFrame->getPose().isValid()==false)
			return false;
	}

	return true;
}


bool
MapBuilder2::checkDataPoints (GenericDataset::ConstPtr sourceDs, const dataItemId startPos, const dataItemId stopPos)
const
{
	for (auto currentId=startPos; currentId<=stopPos; ++currentId) {
		auto curFrame = sourceDs->get(currentId);
		if (curFrame->getPose().isValid()==false)
			return false;
	}

	return true;
}


void MapBuilder2::addCameraParam (const CameraPinholeParams &c)
{
	if (c.height==-1 or c.width==-1)
		throw runtime_error("Invalid camera parameter");

	cMap->addCameraParameter(c);
}


void
MapBuilder2::mapPointCulling()
{
	cout << "Culling points..." << flush;

	vector<mpid> mpToRemove;
	const int minKfRelatedFromMP = 3;
	auto allMapPoints = cMap->allMapPoints();
	const int N = allMapPoints.size();

	for (auto &mp: allMapPoints) {

		auto relatedKfs = cMap->getRelatedKeyFrames(mp);
		if (relatedKfs.size() < minKfRelatedFromMP) {
			mpToRemove.push_back(mp);
		}
	}

	cout << "Detected " << mpToRemove.size() << " points\n" << flush;

	cMap->removeMapPointsBatch(mpToRemove);

	cout << "Removed " << mpToRemove.size() << " out of " << N << " points" << flush;
}


void
MapBuilder2::setMask(const cv::Mat &m)
{
	mask = m.clone();
	cMap->setMask(mask);
}


void
MapBuilder2::simulateOpticalFlow(GenericDataset::ConstPtr sourceDs, dataItemId startPos, dataItemId stopPos)
{
	if (startPos==std::numeric_limits<dataItemId>::max() and stopPos==std::numeric_limits<dataItemId>::max()) {
		startPos = 0;
		stopPos = sourceDs->size()-1;
	}

	cMap->reset();

	// First keyframe
	auto frameItem = sourceDs->get(startPos);
	InputFrame kf1 = createInputFrameX(frameItem);

	for (dataItemId ix=startPos+1; ix<=stopPos; ix++) {
		// XXX: Unfinished
		InputFrame cInpFrame = createInputFrameX(sourceDs->get(ix));

	}
}


void
MapBuilder2::runFromDataset2
(GenericDataset::Ptr sourceDs, dataItemId startPos, dataItemId stopPos)
{

}


void
MapBuilder2::visualOdometry
(GenericDataset::Ptr sourceDs, dataItemId startPos, dataItemId stopPos, Trajectory &voResult)
{
	assert (startPos<sourceDs->size()-1);
	assert (0<stopPos and stopPos<sourceDs->size());

	cv::Mat mask = sourceDs->getMask();

	VisualOdometryViewer voViewer;

	list<Tracklet> currentTrackedPoints;
	list<BaseFrame::Ptr> currentActiveFrame;
	list<BaseFrame::Ptr> wallFrame;

	auto anchor = sourceDs->getAsFrame(startPos);
	anchor->computeFeatures(cMap->getFeatureDetector(), mask);
	voResult.clear();
	Pose
		anchorPose = Pose::Identity(),
		curFramePose;
	voResult.push_back( PoseStamped(anchorPose, sourceDs->get(startPos)->getTimestamp()) );
	voViewer.update(anchor);
	currentActiveFrame.push_back(anchor);

	for (dataItemId d=startPos+1; d<=stopPos; ++d) {
		auto curFrame = sourceDs->getAsFrame(d);
		curFrame->setPose(Pose::Identity());
		curFrame->computeFeatures(cMap->getFeatureDetector(), mask);

		vector<Matcher::KpPair> featurePairs12, validKpPair;
		TTransform T12;
		Matcher::matchAny(*anchor, *curFrame, featurePairs12, cMap->getDescriptorMatcher());
		T12 = Matcher::calculateMovement(*anchor, *curFrame, featurePairs12, validKpPair);

		cerr << "Found " << validKpPair.size() << " pairs\n";

		// Find translation scale
		double theta, phi;
		Matcher::rotationFinder(*anchor, *curFrame, featurePairs12, theta, phi);
		const double __L__ = 2.1;
		double lambda =  -2*__L__*sin(theta/2) / sin(theta/2 - phi);

		T12.translation() *= lambda;

		curFramePose = anchorPose * T12;
		PoseStamped curFrp(curFramePose, sourceDs->get(d)->getTimestamp());

		voViewer.update(curFrame, anchor, validKpPair);

		voResult.push_back(curFrp);
		anchor = curFrame;
		anchorPose = curFramePose;
		cerr << d << endl;

	}
}


VisualOdometryViewer::VisualOdometryViewer()
{
	cv::namedWindow("VO");
}


VisualOdometryViewer::~VisualOdometryViewer()
{
	cv::destroyWindow("VO");
}


void
VisualOdometryViewer::update
	(BaseFrame::ConstPtr currentFrame, BaseFrame::ConstPtr prevFrame,
	const std::vector<Matcher::KpPair> &matchPairs)
{
	if (prevFrame==nullptr)
		cv::imshow("VO", currentFrame->getImage());
	else {
		cv::Mat matchingImg = Matcher::drawMatches(*prevFrame, *currentFrame, matchPairs, Matcher::DrawOpticalFlow);
		cv::imshow("VO", matchingImg);
	}
	cv::waitKey(1);
}


void
VisualOdometryViewer::updateOnlyFeatures(const BaseFrame::ConstPtr frame, const std::vector<cv::KeyPoint> &desiredKeypoints)
{
	cv::Mat curImgWithFeats;

	if (desiredKeypoints.size()==0)
		cv::drawKeypoints(frame->getImage(), frame->allKeypoints(), curImgWithFeats, cv::Scalar(0,255,0));
	else
		cv::drawKeypoints(frame->getImage(), desiredKeypoints, curImgWithFeats, cv::Scalar(0,255,0));

	cv::imshow("VO", curImgWithFeats);
	cv::waitKey(1);
}
