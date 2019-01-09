/*
 * GenericDataset.cpp
 *
 *  Created on: Aug 3, 2018
 *      Author: sujiwo
 */

#include <fstream>
#include "datasets/GenericDataset.h"

using namespace std;
using namespace Eigen;


string GenericDataset::dSetName = "Generic";


void
GenericDataset::dump(const std::string &filename)
{
	const char dumpSep = ',';
	ostream *fd;
	ofstream fdr;
	if (filename.size()==0)
		fd = &cout;
	else {
		fdr.open(filename);
		if (fdr.good())
			throw runtime_error("Unable to open file");
		fd = &fdr;
	}

	*fd << fixed;
	*fd << setprecision(4);

	for (int i=0; i<this->size(); i++) {
		auto di = this->get(i);
		Vector3d pos = di->getPosition();
		Quaterniond orn = di->getOrientation();
		*fd << di->getId()
			<< dumpSep << pos.x()
			<< dumpSep << pos.y()
			<< dumpSep << pos.z()
			<< dumpSep << orn.x()
			<< dumpSep << orn.y()
			<< dumpSep << orn.z()
			<< dumpSep << orn.w()
			<< endl;
	}

	if (fdr.is_open())
		fdr.close();
}


double
GenericDataset::length() const
{
	ptime t0 = get(0)->getTimestamp();
	ptime tlast = get(size()-1)->getTimestamp();

	return double((tlast - t0).total_microseconds()) / 1e6;
}


float
GenericDataset::hertz() const
{
	auto duration = length();
	return float(size()) / length();
}


void GenericDataset::convertStartDurationToTime
(const double startTimeSec, const double duration, ptime &start, ptime &stop)
const
{
	auto recordingLength = this->length();
	if (startTimeSec > recordingLength or startTimeSec+duration > recordingLength)
		throw runtime_error("Time conversion error: outside dataset range");

	tduration
		tstart = boost::posix_time::microsec(startTimeSec * 1e6),
		tdstop = boost::posix_time::microsec(duration * 1e6);

	start = this->get(0)->getTimestamp() + tstart;
	stop = start + tdstop;
}


size_t
GenericDataset::size(const ptime &start, const ptime &stop) const
{
	auto n1 = getLowerBound(start);
	auto n2 = getLowerBound(stop);
	return n2 - n1;
}


BaseFrame::Ptr GenericDataset::getAsFrame(dataItemId i) const
{
	auto diCurrent = this->get(i);
	return BaseFrame::create(diCurrent->getImage(), diCurrent->getPose(), this->getCameraParameter());
}
