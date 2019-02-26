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
 
#include "pointcloudXYZIRAD.h"

namespace velodyne_pointcloud
{
  void PointcloudXYZIRAD::addPoint(const float& x, const float& y, const float& z, const uint16_t& ring, const uint16_t& azimuth, const float& distance, const float& intensity)
  {
    velodyne_pointcloud::PointXYZIRAD point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = intensity;
    point.ring = ring;
    point.azimuth = azimuth;
    point.distance = distance;

    pc->points.push_back(point);
    ++pc->width;
  }
}
