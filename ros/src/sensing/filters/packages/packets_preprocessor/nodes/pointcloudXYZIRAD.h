/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __POINTCLOUDXYZIRAD_H
#define __POINTCLOUDXYZIRAD_H

#include <pcl_ros/point_cloud.h>
#include <velodyne_pointcloud/datacontainerbase.h>

#include "point_types.h"

namespace velodyne_pointcloud
{
  class PointcloudXYZIRAD : public velodyne_rawdata::DataContainerBase
  {
  public:
    pcl::PointCloud<velodyne_pointcloud::PointXYZIRAD>::Ptr pc;

    PointcloudXYZIRAD() : pc(new pcl::PointCloud<velodyne_pointcloud::PointXYZIRAD>) {}

    virtual void addPoint(const float& x, const float& y, const float& z, const uint16_t& ring, const uint16_t& azimuth, const float& distance, const float& intensity);
  };
}
#endif
