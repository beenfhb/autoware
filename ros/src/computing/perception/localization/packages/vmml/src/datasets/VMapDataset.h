/*
 * VMapDataset.h
 *
 *  Created on: Feb 8, 2019
 *      Author: sujiwo
 */

#ifndef _VMAPDATASET_H_
#define _VMAPDATASET_H_

#include <string>
#include <memory>

#include "VMap.h"
#include "utilities.h"


class VMapDataItem: public GenericDataItem
{

};


class VMapDataset : public GenericDataset
{
public:
	VMapDataset (const std::string &mapFilename);
	virtual ~VMapDataset();

protected:
	std::shared_ptr<VMap> srcMap;
	GenericDataset::Ptr sourceDataset;
};

#endif /* _VMAPDATASET_H_ */
