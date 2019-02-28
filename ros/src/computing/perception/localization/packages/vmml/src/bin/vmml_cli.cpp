/*
 * test_localizer.cpp
 *
 *  Created on: Aug 29, 2018
 *      Author: sujiwo
 */



#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <histedit.h>
#include <unistd.h>
#include <editline/readline.h>
#include <opencv2/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <boost/filesystem.hpp>

#include "VMap.h"
#include "KeyFrame.h"
#include "ImageDatabase.h"
#include "SequenceSLAM.h"
#include "Localizer.h"
#include "MapBuilder2.h"
#include "Viewer.h"
#include "Matcher.h"
#include "datasets/OxfordDataset.h"
#include "datasets/MeidaiBagDataset.h"
#include "utilities.h"
#include "Optimizer.h"


using namespace std;
namespace fs = boost::filesystem;


struct {
	std::string velodyneCalibrationPath;
	std::string pcdMapPath;

	// XXX: Find a way to specify these values from external input
	TTransform lidarToCamera = defaultLidarToCameraTransform;

} meidaiNdtParameters;


// XXX: Find a way to specify these values from external input
CameraPinholeParams meidaiCamera1Params(
	1150.96938467,	// fx
	1150.96938467,	// fy
	988.511326762,	// cx
	692.803953253,	// cy
	1920,			// width
	1440			// height
);


typedef vector<string> stringTokens;


const string TestPrompt = "OX>";


class LineEditor
{
public:
	LineEditor(const char *argv0, const string &askPrompt="")
	{
		prompt = askPrompt + ' ';
		el = el_init("test_localizer", stdin, stdout, stderr);
		rl_initialize();
		tokenizer = tok_init(NULL);
	}


	~LineEditor()
	{
		tok_end(tokenizer);
		el_end(el);
	}


	stringTokens parseLine (const char *sline)
	{
		int nt;
		const char **stoklist;
		tok_str(tokenizer, sline, &nt, &stoklist);

		stringTokens st;
		for (int i=0; i<nt; i++) {
			string s(stoklist[i]);
			st.push_back(s);
		}
		tok_reset(tokenizer);

//		free(stoklist);
		return st;
	}


	stringTokens getLine ()
	{
		char *sline = readline(prompt.c_str());
		stringTokens st = parseLine(sline);
		free(sline);

		return st;
	}

protected:
	EditLine *el;
	Tokenizer *tokenizer;
	string prompt;
};


int localize_seq_slam (SequenceSLAM *seqSl, OxfordImagePreprocessor &proc, const string &imgPath)
{
	cv::Mat image = proc.load(imgPath);
	cv::imwrite("/tmp/1.png", image);
	exit(-1);

	vector<cv::Mat> imgList;
	imgList.push_back(image);

	seqSl->find(imgList, 10);
}




class VmmlCliApp
{
public:
	enum datasetType {
		OXFORD_DATASET_TYPE,
		MEIDAI_DATASET_TYPE
	} ;


	struct InputOffsetPosition {
		double asSecondsFromStart = -1;
		int64_t asPosition = -1;

		static InputOffsetPosition
		parseString(const string &s)
		{
			InputOffsetPosition inp;
			if (s.find('.') != string::npos) {
				inp.asSecondsFromStart = stod(s);
			}
			else {
				inp.asPosition = stoi(s);
			}
			return inp;
		}

	};


	VmmlCliApp (int argc, char *argv[]):
		mLineEditor(argv[0], TestPrompt),
		mapPath("")

	{
	//	localizer = new Localizer()
	}


	~VmmlCliApp ()
	{
		if (mapSrc)
			delete(mapSrc);
		if (localizer)
			delete(localizer);
	}


	void processCommand (const stringTokens &command)
	{
		if (command[0][0]=='#' or command[0].size()==0)
			return;

		else if (command[0]=="map")
			{ RecordRuntime("MapOpen", map_open_cmd(command[1]) ); }

		else if (command[0]=="dataset")
			{ RecordRuntime("DatasetOpen", dataset_open_cmd(command)); }

		else if (command[0]=="map_pcl")
			map_dump_pcl();

		else if (command[0]=="dataset_trajectory")
			dataset_trajectory_dump(command[1]);

		else if (command[0]=="map_trajectory")
			map_trajectory_dump();

//			else if (command[0]=="find")
//				map_find_cmd(command[1]);

		else if (command[0]=="save")
			dataset_save_dsecond(command[1]);

		else if (command[0]=="savei")
			dataset_save_id(command[1]);

		else if (command[0]=="zoom")
			dataset_set_zoom(command[1]);

		else if (command[0]=="dataset_view")
			dataset_view(command[1]);

		else if (command[0]=="detect")
			{ RecordRuntime("PlaceDetection", map_detect_cmd(command[1]) ); }

		// To ask a subset, specify `start' and `stop' offset from beginning
		// as optional parameters
		else if (command[0]=="map_create")
			{ RecordRuntime("MapCreate", map_create_cmd(stringTokens(command.begin()+1, command.end())) ); }

		else if (command[0]=="map_info")
			map_info_cmd();

		else if (command[0]=="dataset_info")
			dataset_info_cmd();

		else if (command[0]=="mask")
			mask_set_cmd(command[1]);

		else if (command[0]=="velodyne" or command[0]=="pcdmap")
			dataset_set_param(command);

		else if (command[0]=="build")
			{ RecordRuntime("DatasetBuild", dataset_build(command) ); }

		else if (command[0]=="map_images")
			map_dump_images();

		else if (command[0]=="cd" or command[0]=="chdir")
			changeWorkingDirectory(command[1]);

		else if (command[0]=="match")
			match_frames_cmd(command);

		else if (command[0]=="project")
			projection_cmd(command);

		else if (command[0]=="vo")
			vo_cmd(command);

		else if (command[0]=="camera_baselink_offset")
			camera_baselink_offset_cmd(command);

		else if (command[0]=="map_ba")
			map_ba_cmd();

		else if (command[0]=="simulate_features")
			simulate_features_cmd(command);

		else if (command[0]=="homography")
			homography_cmd(command);
	}


	static void fromScript(int argc, char *argv[])
	{
		VmmlCliApp scriptIterpreter (argc, argv);
		string scriptName(argv[1]);

		std::ifstream fd;
		fd.open(scriptName);
		if (fd.good()==false)
			throw std::runtime_error("Unable to open file "+scriptName);

		while(fd.eof()==false) {
			std::string curLine;
			std::getline(fd, curLine);

			if (curLine[0]=='#')
				continue;

			stringTokens command = scriptIterpreter.mLineEditor.parseLine(curLine.c_str());
			scriptIterpreter.processCommand(command);
		}

		cerr << "\nDone\n";
	}


	void loop()
	{
		bool doExit=false;
		while (doExit==false) {

			stringTokens command = mLineEditor.getLine();

			if (command[0]=="quit")
				break;

			processCommand(command);
		}
	}


protected:
	LineEditor mLineEditor;

	VMap *mapSrc = NULL;
	boost::filesystem::path mapPath;

	ImageDatabase *imgDb = NULL;
	SequenceSLAM *seqSlProv = NULL;
	Localizer *localizer = NULL;

	datasetType slDatasourceType;
	GenericDataset::Ptr loadedDataset;
	MeidaiBagDataset::Ptr meidaiDsPtr = nullptr;
	OxfordDataset::Ptr oxfordDsPtr = nullptr;
	boost::filesystem::path datasetPath;

	cv::Mat mask;


private:


	void changeWorkingDirectory (const string &toDir)
	{
		chdir(toDir.c_str());
		return;
	}


	void map_open_cmd(const string &mapPathInput)
	{
		try {
			mapSrc = new VMap();
			mapSrc->load(mapPathInput);
			localizer = new Localizer(mapSrc);
			localizer->setCameraParameterFromId(0);

			debug("Map loaded");
			imgDb = mapSrc->getImageDB();
			seqSlProv = imgDb->getSequence();
			mapPath = boost::filesystem::path(mapPathInput);

			mask = mapSrc->getMask();
			if (mask.empty()==false) {
				debug ("Map contains mask image file");
				localizer->setMask(mask);
			}

		} catch (exception &e) {
			debug ("Unable to load map");
		}
	}


	const string viewerWindowName="Dataset Viewer";
	void dataset_view(const string &durationSecStr)
	{
		cv::namedWindow(viewerWindowName);
		double d = std::stod(durationSecStr);
		auto di = loadedDataset->atDurationSecond(d);
		cv::Mat img = di->getImage();
		cv::cvtColor(img, img, CV_RGB2GRAY);
		cv::imshow(viewerWindowName, img);
		cv::waitKey(1);
	}

	const string mapDumpPcl = "/tmp/map.pcd";
	void map_dump_pcl()
	{
		auto pclDump = mapSrc->dumpPointCloudFromMapPoints();
		pcl::io::savePCDFileBinary(mapDumpPcl, *pclDump);
		debug("Point Cloud Map dumped to "+mapDumpPcl);
	}


	/*
	 * Building a `ground truth' for Meidai dataset accepts start and stop time,
	 * expressed in seconds after recording started
	 * Syntax:
	 * build [startOffset] [stopOffset] [gnss]
	 */
	void dataset_build(const stringTokens &cmd)
	{
		if (slDatasourceType!=MEIDAI_DATASET_TYPE) {
			debug("Oxford datasets need not to be built");
			return;
		}

		ptime
			t1 = MIN_TIME,
			t2 = MAX_TIME;

		bool useLidar=true;
		if (cmd.size() == 2) {
			if (cmd[1]=="gnss")
				useLidar = false;
		}

		else if (cmd.size()>=3) {

			double startPos = stod(cmd[1]),
				stopPos = stod(cmd[2]);

			if (cmd.size()==4 and cmd[3]=="gnss")
				useLidar = false;

			debug ("Building from "+to_string(startPos) + " to " + to_string(stopPos));
			t1 = meidaiDsPtr->timeFromStart(startPos);
			t2 = meidaiDsPtr->timeFromStart(stopPos);
//			meidaiDsPtr->setTimeConstraint(t1, t2);
		}

		if (useLidar==false) {
			debug ("Not using NDT; camera positions are estimated from GNSS");
		}

		else if (useLidar==true) {
			if (meidaiNdtParameters.pcdMapPath.empty()) {
				debug ("Point cloud map must be set with commands `pcdmap'");
				return;
			}

			meidaiNdtParameters.lidarToCamera = defaultLidarToCameraTransform;
			string velodyneCalibrationPath;
			if (meidaiNdtParameters.velodyneCalibrationPath.empty()) {
				boost::filesystem::path myPath = getMyPath();
				myPath /= "params/meidai-64e-S2.yaml";
				velodyneCalibrationPath = myPath.string();
			}
			else
				velodyneCalibrationPath = meidaiNdtParameters.velodyneCalibrationPath;

			meidaiDsPtr->setLidarParameters(velodyneCalibrationPath, meidaiNdtParameters.pcdMapPath, meidaiNdtParameters.lidarToCamera);
		}

		meidaiDsPtr->forceCreateCache(useLidar, t1, t2);

		debug ("Done");
	}


	void map_info_cmd()
	{
		if (mapSrc==NULL) {
			debug("Map not loaded");
			return;
		}
		debug("# of keyframe(s): "+to_string(mapSrc->numOfKeyFrames()));
		debug("# of map point(s): " +to_string(mapSrc->numOfMapPoints()));

		auto camInfo = mapSrc->getCameraParameter(0);
		debug("Horizontal FieldOfView (rad): " + to_string(camInfo.getHorizontalFoV()));
		debug("Vertical FieldOfView (rad): " + to_string(camInfo.getVerticalFoV()));

		auto &mapInfo = mapSrc->getAllInfo();
		for (auto rInfo: mapInfo) {
			const string
				&k = rInfo.first,
				&v = rInfo.second;
			stringstream ss;
			ss << k << " : " << v;
			debug(ss.str());
		}
	}


	void dataset_info_cmd()
	{
		if (loadedDataset==nullptr) {
			debug("No dataset loaded");
			return;
		}

		if (slDatasourceType==MEIDAI_DATASET_TYPE) {
			auto cameraTrack = meidaiDsPtr->getCompleteCameraTrajectory();

			if (cameraTrack.empty()==false) {

				if (meidaiDsPtr->isCameraTrajectoryComplete()) {
					debug("Camera trajectory is complete");
				}
				else
					debug("Camera trajectory is partial");

				ptime time0 = meidaiDsPtr->get(0)->getTimestamp();
				auto
					td1 = toSeconds(cameraTrack.front().timestamp - time0),
					td2 = toSeconds(cameraTrack.back().timestamp - time0);
				debug("Available time positions: from " + to_string(td1) + " to " + to_string(td2));

				switch (meidaiDsPtr->cameraTrackSource) {
				case MeidaiBagDataset::GNSS: debug("Camera trajectory is derived from GNSS"); break;
				case MeidaiBagDataset::NDT: debug("Camera trajectory is derived from NDT"); break;
				case MeidaiBagDataset::ICP: debug("Camera trajectory is derived from ICP"); break;
				}
			}

			else {
				debug ("Dataset does not contain camera trajectory; must be built");
			}
		}

		debug("# of images:" + to_string(loadedDataset->size()));

		debug("Frequency (Hz): " + to_string(loadedDataset->hertz()));

		debug("Duration (sec): " + to_string(loadedDataset->length()));

		debug("Zoom: " + to_string(loadedDataset->getZoomRatio()));

		// Image size
		auto cam = loadedDataset->getCameraParameter();
		debug("Image width: " + to_string(cam.width));
		debug("Image height: " + to_string(cam.height));
	}


	const string dumpMapTrajectoryPath = "/tmp/dump_map_trajectory.csv";
	void map_trajectory_dump()
	{
		Trajectory mapTrack;
		mapSrc->dumpCameraPoses(mapTrack);
		mapTrack.dump(dumpMapTrajectoryPath);

		debug("Map trajectory dumped to "+dumpMapTrajectoryPath);
	}


	const string dumpDatasetTrajectoryPath = "/tmp/dump_dataset_trajectory";
	void dataset_trajectory_dump(const string &type="camera")
	{
		Trajectory dsTrack;
		string dumpPathName;

		if (slDatasourceType==MEIDAI_DATASET_TYPE) {

			if (type=="gnss")
				dsTrack = meidaiDsPtr->getGnssTrajectory();

			else if (type=="ndt")
				dsTrack = meidaiDsPtr->getNdtTrajectory();

			else if (type=="camera") {
				dsTrack = meidaiDsPtr->getCameraTrajectory();

				/*for (int i=0; i<dsTrack.size(); i++) {
					auto P = dsTrack.at(i);
					P = P.shift(Vector3d(3.0, 0, 0));
					dsTrack[i] = P;
				}*/
			}

			else {
				debug ("Unknown trajectory type for Meidai Dataset");
				return;
			}

			dumpPathName = dumpDatasetTrajectoryPath + '-' + type + ".csv";
		}

		else if(slDatasourceType==OXFORD_DATASET_TYPE) {
			dsTrack = loadedDataset->getCameraTrajectory();
			dumpPathName = dumpDatasetTrajectoryPath + '-' + fs::basename(oxfordDsPtr->getPath());
		}

		dsTrack.dump(dumpPathName);
		debug("Dataset trajectory dumped to "+dumpPathName);
		return;
	}


//	void dataset_open_cmd(const string &dsPath, const string &modelDir)
	void dataset_open_cmd(const stringTokens &cmd)
	{
		datasetPath = boost::filesystem::path (cmd[1]);

		if (boost::filesystem::is_directory(datasetPath)) {

			if (cmd.size()==2) {
				debug("Oxford SDK Model Directory not specified");
				return;
			}

			loadedDataset = OxfordDataset::load(datasetPath.string(), cmd[2]);
			slDatasourceType = OXFORD_DATASET_TYPE;
			oxfordDsPtr = static_pointer_cast<OxfordDataset> (loadedDataset);
			debug ("Oxford-type Dataset Loaded");
		}

		else if (datasetPath.extension()==".bag") {
			loadedDataset = MeidaiBagDataset::load(datasetPath.string());
			meidaiDsPtr = static_pointer_cast<MeidaiBagDataset>(loadedDataset);
			slDatasourceType = MEIDAI_DATASET_TYPE;
			meidaiDsPtr->addCameraParameter(meidaiCamera1Params);
			meidaiDsPtr->isPreprocessed = true;
			debug ("Nagoya University Dataset Loaded");
		}

		else {
			debug("Unsupported dataset type");
			return;
		}
	}


	void dataset_set_param(const stringTokens &command)
	{
		if (slDatasourceType==MEIDAI_DATASET_TYPE) {
			if (command[0]=="velodyne")
				meidaiNdtParameters.velodyneCalibrationPath = command[1];
			else if (command[0]=="pcdmap")
				meidaiNdtParameters.pcdMapPath = command[1];
		}
	}


	/*
	 * You can feed an offset number from dataset, or a path to an image
	 */
	void map_detect_cmd(const string &detectionStr)
	{
		if (localizer==NULL) {
			debug("Map not loaded");
			return;
		}

		cv::Mat img;

		if (boost::filesystem::exists(boost::filesystem::path(detectionStr))) {
			img = cv::imread(detectionStr, cv::IMREAD_COLOR);
			if (img.empty()) {
				debug ("Unable to open requested file");
				return;
			}
		}

		else {
			double d = std::stod(detectionStr);
			auto di = loadedDataset->atDurationSecond(d);
			img = di->getImage();
		}

		cv::cvtColor(img, img, CV_RGB2GRAY);
		kfid kmap;
		Pose computedPose;
		bool gotplace = localizer->detect_mt(img, kmap, computedPose);

		debug("Detecting " + detectionStr);
		if (gotplace) {
			debug(computedPose.str(true));
			debug("Result: "+to_string(kmap));
		}
		else {
			debug ("Unable to detect place");
		}
	}

	const string dumpImagePath = "/tmp/dump_image.png";

	void dataset_save_dsecond(const string &durationSecStr)
	{
		double d = std::stod(durationSecStr);
		auto di = loadedDataset->atDurationSecond(d);
		cv::Mat img = di->getImage();
		cv::imwrite(dumpImagePath, img);
		debug("Dumped to " + dumpImagePath);
	}

	/*
	 * Save with detected features and Lidar scans
	 * XXX: Unfinished
	 */
	void dataset_save_id(const string &sid)
	{
		dataItemId requestId = static_cast<dataItemId>(std::stoi(sid));

		auto md = loadedDataset->get(requestId);
		cv::Mat img = md->getImage().clone();

		if (mask.empty()==false) {

		}
		else {

		}

		cv::imwrite(dumpImagePath, img);
		debug("Image #" + sid + " dumped to " + dumpImagePath);
	}

	void debug(const string &s, double is_error=false)
	{
		if (!is_error)
			cerr << s << endl << flush;
		else
			cout << s << endl << flush;
	}

	void dataset_set_zoom(const string &zstr)
	{
		float z = std::stof(zstr);
		loadedDataset->setZoomRatio(z);
	}

	string createMapFilename ()
	{
		string mapResName;

		if (slDatasourceType==OXFORD_DATASET_TYPE) {
			OxfordDataset::Ptr oxfDs = static_pointer_cast<OxfordDataset> (loadedDataset);
			mapResName = oxfDs->getPath() + "/vmml.map";
		}
		else if(slDatasourceType==MEIDAI_DATASET_TYPE) {
			MeidaiBagDataset::Ptr mdiDs = static_pointer_cast<MeidaiBagDataset> (loadedDataset);
			mapResName = mdiDs->getPath() + ".map";
		}
		return mapResName;
	}


	void map_create_cmd (const stringTokens &cmd)
	{
		ptime ptstart, ptstop;
		InputOffsetPosition start, stop;
		double duration;
		int numOfFrames;
		bool
			useAllFrames = false,
			useIntPosition = false;

		MapBuilder2 mapBuilder;
		mapBuilder.setMask(loadedDataset->getMask());

		if (slDatasourceType == OXFORD_DATASET_TYPE) {
			mapBuilder.getMap()->setInfo("sourceType", "Oxford");
			mapBuilder.getMap()->setInfo("originalPath", oxfordDsPtr->getPath());
		}

		else if (slDatasourceType == MEIDAI_DATASET_TYPE) {
			mapBuilder.getMap()->setInfo("sourceType", "Meidai");
			mapBuilder.getMap()->setInfo("originalPath", meidaiDsPtr->getPath());
		}

		if (cmd.size() >= 1) {
			start = InputOffsetPosition::parseString(cmd[0]);
			if (cmd.size()>=2)
				stop = InputOffsetPosition::parseString(cmd[1]);
			else
				stop = InputOffsetPosition();

			if (start.asPosition==-1 and stop.asPosition==-1) {
				duration = stop.asSecondsFromStart - start.asSecondsFromStart;
				loadedDataset->convertStartDurationToTime(start.asSecondsFromStart, duration, ptstart, ptstop);
				numOfFrames = loadedDataset->size(ptstart, ptstop);
				useIntPosition = false;
			}

			else if (start.asPosition!=-1 and stop.asPosition==-1) {
				stop.asPosition = loadedDataset->size()-1;
				try {
					ptstart = loadedDataset->get(start.asPosition)->getTimestamp();
					ptstop  = loadedDataset->last()->getTimestamp();
					duration = toSeconds(ptstop-ptstart);
					numOfFrames = (loadedDataset->size()-1) - start.asPosition;
					useIntPosition = true;
				} catch(exception &e) {
					debug("Requested position(s) does not exist");
					return;
				}
			}

			else if(start.asSecondsFromStart==-1 and stop.asSecondsFromStart==-1) {
				try {
					ptstart = loadedDataset->get(start.asPosition)->getTimestamp();
					ptstop  = loadedDataset->get(stop .asPosition)->getTimestamp();
					duration = toSeconds(ptstop-ptstart);
					numOfFrames = stop.asPosition - start.asPosition;
					useIntPosition = true;
				} catch(exception &e) {
					debug("Requested position(s) does not exist");
					return;
				}
			}

			else {
				debug("Invalid position specification");
				return;
			}
		}

		else {
			ptstart = loadedDataset->get(0)->getTimestamp();
			ptstop = loadedDataset->last()->getTimestamp();
			duration = toSeconds((ptstop-ptstart));
			numOfFrames = loadedDataset->size();
			useAllFrames = true;
		}

		debug ("About to run mapping with duration "+to_string(duration) +" seconds, " +to_string(numOfFrames) + " frames");

		// build map here
		mapBuilder.addCameraParam(loadedDataset->getCameraParameter());

		Viewer *imgViewer = new Viewer (loadedDataset);
		imgViewer->setMap(mapBuilder.getMap());
		dataItemId currentItemId;
		int _callbackFrameId = 1;

		/* KeyFrame Callback */
		MapBuilder2::frameCallback frmCallback =
		[&] (const InputFrame &f)
		{
			imgViewer->update(f.sourceId, mapBuilder.getCurrentKeyFrameId());
//			cout << _callbackFrameId << " / " << numOfFrames << endl;
			_callbackFrameId += 1;
		};
		mapBuilder.registerFrameCallback(frmCallback);

		if (useAllFrames==true) {
			mapBuilder.runFromDataset(loadedDataset, numeric_limits<dataItemId>::max(), numeric_limits<dataItemId>::max());
		}
		else {
			if (useIntPosition==true) {
				if (slDatasourceType==MEIDAI_DATASET_TYPE) {
					mapBuilder.runFromDataset(meidaiDsPtr, start.asPosition, stop.asPosition);
				}
				else {
					mapBuilder.runFromDataset(loadedDataset, start.asPosition, stop.asPosition);
				}
			}
			else
				mapBuilder.runFromDataset(loadedDataset, ptstart, ptstop);
		}

		delete(imgViewer);
		// Stop here

		const string mapFilePath = createMapFilename();
		mapBuilder.getMap()->save(mapFilePath);

		debug ("Mapping done");
		debug ("Dataset time elapsed: " + to_string(duration) + " seconds");
		debug ("Path: " + mapFilePath);
	}


	void mask_set_cmd(const string &maskpath)
	{
		mask = cv::imread(maskpath, cv::IMREAD_GRAYSCALE);
		if (mask.empty()) {
			debug("Unable to fetch mask image file");
			return;
		}

		if (loadedDataset) {
			float zr = loadedDataset->getZoomRatio();
			cv::resize(mask, mask, cv::Size(), zr, zr, cv::INTER_CUBIC);
		}

		debug("Mask read; size set to "+to_string(mask.cols)+'x'+to_string(mask.rows));
	}


	void map_dump_images()
	{
		if (localizer==NULL) {
			debug ("Map not loaded");
			return;
		}

		MeidaiBagDataset::Ptr meidaiDs;
		auto originalPath = mapSrc->getInfo("originalPath");
		try {
			meidaiDs = MeidaiBagDataset::load(originalPath);
		} catch (exception &e) {
			debug ("Unable to open dataset "+originalPath);
			return;
		}

		boost::filesystem::path dumpDir = mapPath.parent_path() /= "/mapdump";
		boost::filesystem::create_directory(dumpDir);

		for (auto &kfI: mapSrc->allKeyFrames()) {
			auto srcItemId = mapSrc->keyframe(kfI)->getSourceItemId();
			auto srcDataItem = meidaiDs->get(srcItemId);
			cv::Mat cImage = srcDataItem->getImage();
			string imgNew = dumpDir.string() + '/' + (to_string(srcItemId) + ".jpg");
			cv::imwrite(imgNew, cImage);
		}

		debug ("Map keyframes dumped to " + dumpDir.string());
	}


	// Match Testing
	void match_frames_cmd(const stringTokens &cmd)
	{
		if (cmd.size()<3) {
			debug("Usage: match <frame#1> <frame#2> [mode=o|s]");
			return;
		}

		int
			frnum1 = stoi(cmd[1]),
			frnum2 = stoi(cmd[2]);

		Matcher::DrawMode drawmode;
		if (cmd.size()==3)
			drawmode = Matcher::DrawOpticalFlow;
		else {
			if (cmd[3][0]=='o')
				drawmode = Matcher::DrawOpticalFlow;
			else if (cmd[3][0]=='s')
				drawmode = Matcher::DrawSideBySide;
			else if (cmd[3][0]=='p')
				drawmode = Matcher::DrawOnlyPoints;
			else if (cmd[3][0]=='e')
				drawmode = Matcher::DrawEpipolarIn2;
		}

		int maxNum = -1;
		if (cmd.size()>4) {
			maxNum = stoi(cmd[4]);
			Matcher::__maxDraw = maxNum;
		}

		auto
			Frame1 = loadedDataset->getAsFrame(frnum1),
			Frame2 = loadedDataset->getAsFrame(frnum2);
		/*
		 * Set circle of confusion as uncertainty of 2D points
		 */
		Matcher::circleOfConfusionDiameter = loadedDataset->getCoC();

		// Need map's feature detector and matcher
		MapBuilder2 mpBuilder;
		auto cvFeatDetector = mpBuilder.getMap()->getFeatureDetector();
		auto cvFeatMatcher = mpBuilder.getMap()->getDescriptorMatcher();

		cv::Mat mask = loadedDataset->getMask();
		Frame1->computeFeatures(cvFeatDetector, mask);
		Frame2->computeFeatures(cvFeatDetector, mask);

		vector<Matcher::KpPair> featurePairs, validKpPairs;
		TTransform T12;
		Matcher::matchAny(*Frame1, *Frame2, featurePairs, cvFeatMatcher);
		T12 = Matcher::calculateMovement(*Frame1, *Frame2, featurePairs, validKpPairs);

		cv::Mat matchResult;
		matchResult = Matcher::drawMatches(*Frame1, *Frame2, validKpPairs, drawmode, maxNum);

		const string matchFiledump("match.png");
		cv::imwrite(matchFiledump, matchResult);
		debug("Matching result written to "+matchFiledump);
		debug("Got "+to_string(validKpPairs.size())+" valid pairs");

		double theta, phi;
		Matcher::rotationFinder(*Frame1, *Frame2, validKpPairs, theta, phi);
		const double L = 2.1;
		double lambda =  -2*L*sin(theta/2) / sin(theta/2 - phi);
		debug("lambda: " + to_string(lambda));
		T12.translation() *= lambda;

		// Match by ICP Lidar scans
		auto
			meFrame1 = meidaiDsPtr->getNative(frnum1),
			meFrame2 = meidaiDsPtr->getNative(frnum2);
		TTransform TL12 = Matcher::matchLidarScans(*meFrame1, *meFrame2);

		/*
		 * Statistics
		 */
		debug("Camera transformation by E: ");
		debug(dumpVector(T12));
		debug("Camera metric transformation: ");
		TTransform T12m = Frame1->pose().inverse() * Frame2->pose();
		debug(dumpVector(T12m));
		debug("Lidar transformation: ");
		debug(dumpVector(TL12));
	}


	void homography_cmd(const stringTokens &cmd)
	{
		int
			frnum1 = stoi(cmd[1]),
			frnum2 = stoi(cmd[2]);

		auto
			Frame1 = loadedDataset->getAsFrame(frnum1),
			Frame2 = loadedDataset->getAsFrame(frnum2);

		// Need map's feature detector and matcher
		MapBuilder2 mpBuilder;
		auto cvFeatDetector = mpBuilder.getMap()->getFeatureDetector();
		auto cvFeatMatcher = mpBuilder.getMap()->getDescriptorMatcher();

		Eigen::Matrix3d hMat;
		Matcher::matchH(*Frame1, *Frame2, mask, cvFeatDetector, cvFeatMatcher, hMat);

		cout << "Done homography test" << endl;
	}


	void camera_baselink_offset_cmd(const stringTokens &cmd)
	{
		if (cmd.size()<3) {
			debug("Usage: match <frame#1> <frame#2> [mode=o|s]");
			return;
		}

		int
			frnum1 = stoi(cmd[1]),
			frnum2 = stoi(cmd[2]);

		auto
			Frame1n = meidaiDsPtr->getNative(frnum1),
			Frame2n = meidaiDsPtr->getNative(frnum2);

		auto
			Frame1 = BaseFrame::create(Frame1n->getImage(), Frame1n->getPose(), meidaiDsPtr->getCameraParameter()),
			Frame2 = BaseFrame::create(Frame2n->getImage(), Frame2n->getPose(), meidaiDsPtr->getCameraParameter());
		/*
		 * Set circle of confusion as uncertainty of 2D points
		 */
		Matcher::circleOfConfusionDiameter = loadedDataset->getCoC();

		// Need map's feature detector and matcher
		MapBuilder2 mpBuilder;
		auto cvFeatDetector = mpBuilder.getMap()->getFeatureDetector();
		auto cvFeatMatcher = mpBuilder.getMap()->getDescriptorMatcher();

		cv::Mat mask = loadedDataset->getMask();
		Frame1->computeFeatures(cvFeatDetector, mask);
		Frame2->computeFeatures(cvFeatDetector, mask);

		vector<Matcher::KpPair> validKpPairs;
		Matcher::matchAny(*Frame1, *Frame2, validKpPairs, cvFeatMatcher);

		double theta, phi;
		Matcher::rotationFinder(*Frame1, *Frame2, validKpPairs, theta, phi);

		Pose
			baselink1 = Frame1n->getBaselinkPose(),
			baselink2 = Frame2n->getBaselinkPose();
		double L = Matcher::getCameraBaselinkOffset(baselink1, baselink2, theta, phi);

		debug("Got camera->baselink offset: " + to_string(L));
	}


	// Projection test
	void projection_cmd(const stringTokens &cmd)
	{
		if (cmd.size()<3) {
			debug("Usage: project <frame#1> <pcd file>");
			return;
		}

		int frameNum = stoi(cmd[1]);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcdInput(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCDReader fReader;
		fReader.read(cmd[2], *pcdInput);

		auto currentFrame = loadedDataset->getAsFrame(frameNum);
		cv::Mat frameImageProjection = currentFrame->projectPointCloud(pcdInput, 200);

		const string projectionDump ("projection.png");
		cv::imwrite(projectionDump, frameImageProjection);
		debug("Projection result written to " + projectionDump);
	}

	// Visual Odometry test
	void vo_cmd (const stringTokens &cmd)
	{
		int fn0 = stoi(cmd[1]),
			fn1 = stoi(cmd[2]);

		fs::path voDumpPath("/tmp/visual_odometry.csv");

		Trajectory voTrajectory;
		MapBuilder2 mpBuilder;

		mpBuilder.visualOdometry(loadedDataset, fn0, fn1, voTrajectory);
		voTrajectory.dump(voDumpPath.string());
		debug("Visual odometry result dumped to "+voDumpPath.string());
	}

	// Bundle Adjustment Test
	void map_ba_cmd()
	{
		if (mapSrc==nullptr) {
			debug("Map not loaded");
			return;
		}

		bundle_adjustment_2(mapSrc);
	}

	// Simulate Feature Detection across frames
	void simulate_features_cmd(const stringTokens &cmd)
	{
		int fn0 = stoi(cmd[1]),
			fn1 = stoi(cmd[2]);

		VisualOdometryViewer featureViewer;
		MapBuilder2 mpBuilder;
		auto cvFeatDetector = mpBuilder.getMap()->getFeatureDetector();
		auto cvFeatMatcher = mpBuilder.getMap()->getDescriptorMatcher();

		for (int i=fn0; i<=fn1; ++i) {
			auto framePtr = loadedDataset->getAsFrame(i);

			cv::Mat frameDescriptors;
			vector<cv::KeyPoint> desiredKeypoints;

			if (mask.empty()==false) {
				framePtr->computeFeatures(cvFeatDetector, desiredKeypoints, frameDescriptors, mask);
				cerr << "Found " << desiredKeypoints.size() << " #features\n";
				featureViewer.updateOnlyFeatures(framePtr, desiredKeypoints);
			}
			else {
				framePtr->computeFeatures(cvFeatDetector, loadedDataset->getMask());
				cerr << "Found " << framePtr->numOfKeyPoints() << " #features\n";
				featureViewer.updateOnlyFeatures(framePtr);
			}
		}
	}


};





int main (int argc, char *argv[])
{
	if (argc > 1) {
		VmmlCliApp::fromScript(argc, argv);
	}

	else {
		VmmlCliApp mainApp(argc, argv);
		mainApp.loop();
	}

	return 0;
}
