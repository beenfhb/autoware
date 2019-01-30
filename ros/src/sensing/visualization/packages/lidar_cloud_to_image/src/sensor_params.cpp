
#include "sensor_params.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>
#include <memory>

// This work was inspired on projection_params from I. Bogoslavskyi, C. Stachniss, University of Bonn 
// https://github.com/PRBonn/cloud_to_image.git
// The original license copyright is as follows:
//------------------------------------
// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.
//------------------------------------

template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

namespace cloud_to_image 
{

using std::vector;
using std::string;
using std::upper_bound;
using boost::algorithm::starts_with;

void SensorParams::setSpan(const AngularRange& span_params, const AngularRange::Direction& direction) 
{
  vector<AngularRange> params_vec = {{span_params}};
  this->setSpan(params_vec, direction);
}

void SensorParams::setSpan(const vector<AngularRange>& span_params, const AngularRange::Direction& direction) 
{
  int num_beams = 0;
  for (const auto& span : span_params) {
    num_beams += span.num_beams();
  }
  switch (direction) {
    case AngularRange::Direction::HORIZONTAL:
      _h_span_params = AngularRange(span_params.front().start_angle(),
                                    span_params.back().end_angle(), num_beams);
      _col_angles = fillVector(span_params);
      break;
    case AngularRange::Direction::VERTICAL:
      _v_span_params = AngularRange(span_params.front().start_angle(),
                                  span_params.back().end_angle(), num_beams);
      _row_angles = fillVector(span_params);
      break;
  }
  fillCosSin();
}

vector<Angle> SensorParams::fillVector(const AngularRange& span_params) 
{
  vector<AngularRange> params_vec = {{span_params}};
  return this->fillVector(params_vec);
}

vector<Angle> SensorParams::fillVector(const vector<AngularRange>& span_params) 
{
  vector<Angle> res;
  for (const auto span_param : span_params) {
  	int direction = 1; //increasing
    Angle rad = span_param.start_angle();
    if (span_param.start_angle() > span_param.end_angle()) {
    	direction = -1;
    }
    for (int i = 0; i < span_param.num_beams(); ++i) {
      res.push_back(rad);
      rad += (span_param.step() * direction);
    }
  }
  return res;
}

bool SensorParams::valid() 
{
  bool all_params_valid = _v_span_params.valid() && _h_span_params.valid();
  bool arrays_empty = _row_angles.empty() && _col_angles.empty();
  bool cos_sin_empty = _row_angles_sines.empty() &&
                       _row_angles_cosines.empty() &&
                       _col_angles_sines.empty() && _col_angles_cosines.empty();
  if (!all_params_valid) {
    throw std::runtime_error("Sensor parameters invalid.");
  }
  if (arrays_empty) {
    throw std::runtime_error("Sensor parameters arrays not filled.");
  }
  if (cos_sin_empty) {
    throw std::runtime_error("Projection parameters sin and cos arrays not filled.");
  }
  return true;
}

const Angle SensorParams::angleFromRow(int row) const 
{
  if (row >= 0 && static_cast<size_t>(row) < _row_angles.size()) {
    return _row_angles[row];
  }
  fprintf(stderr, "ERROR: row %d is wrong\n", row);
  return 0.0_deg;
}

const Angle SensorParams::angleFromCol(int col) const 
{
  int actual_col = col;
  if (col < 0) {
    actual_col = col + _col_angles.size();
  } else if (static_cast<size_t>(col) >= _col_angles.size()) {
    actual_col = col - _col_angles.size();
  }
  // everything is normal
  return _col_angles[actual_col];
}

size_t SensorParams::rowFromAngle(const Angle& angle) const 
{
  return findClosest(_row_angles, angle);
}

size_t SensorParams::colFromAngle(const Angle& angle) const 
{
  size_t col = findClosest(_col_angles, angle);
  //handle CW or CCW direction
  if (_scan_direction == ScanDirection::CLOCK_WISE) {
    col = (_col_angles.size() - 1) - col; //flip the columns
  }
  return col;
}

size_t SensorParams::findClosest(const vector<Angle>& vec, const Angle& val) 
{
  size_t found = 0;
  if (vec.front() < vec.back()) {
    found = upper_bound(vec.begin(), vec.end(), val) - vec.begin();
  } else {
    found = vec.rend() - upper_bound(vec.rbegin(), vec.rend(), val);
  }
  if (found == 0) {
    return found;
  }
  if (found == vec.size()) {
    return found - 1;
  }
  auto diff_next = Angle::abs(vec[found] - val);
  auto diff_prev = Angle::abs(val - vec[found - 1]);
  return diff_next < diff_prev ? found : found - 1;
}

std::unique_ptr<SensorParams> SensorParams::VLP_16() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(15_deg, -15_deg, 16),
                 AngularRange::Direction::VERTICAL);
  params.setScanDirection("CW");
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::HDL_32() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(10.0_deg, -30.0_deg, 32),
                 AngularRange::Direction::VERTICAL);
  params.setScanDirection("CW");
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::HDL_64_EQUAL() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(2.0_deg, -24.0_deg, 64),
                 AngularRange::Direction::VERTICAL);
  params.setScanDirection("CW");
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::HDL_64() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  AngularRange span_top(2.0_deg, -8.5_deg, 32);
  AngularRange span_bottom(-8.87_deg, -24.87_deg, 32);
  vector<AngularRange> spans = {{span_top, span_bottom}};
  params.setSpan(spans, AngularRange::Direction::VERTICAL);
  params.setScanDirection("CW");
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::OS_1_16_0512() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 512),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 16),
                 AngularRange::Direction::VERTICAL);
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::OS_1_16_1024() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 1024),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 16),
                 AngularRange::Direction::VERTICAL);
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::OS_1_16_2048() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 2048),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 16),
                 AngularRange::Direction::VERTICAL);
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::OS_1_64_0512() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 512),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 64),
                 AngularRange::Direction::VERTICAL);
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::OS_1_64_1024() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 1024),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 64),
                 AngularRange::Direction::VERTICAL);
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::OS_1_64_2048() 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, 2048),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 64),
                 AngularRange::Direction::VERTICAL);
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::fullSphere(const Angle& discretization) 
{
  auto params = SensorParams();
  params.setSpan(AngularRange(-180_deg, 180_deg, discretization),
                 AngularRange::Direction::HORIZONTAL);
  params.setSpan(AngularRange(-90_deg, 90_deg, discretization),
                 AngularRange::Direction::VERTICAL);
  params.fillCosSin();
  if (!params.valid()) {
    fprintf(stderr, "ERROR: params are not valid!\n");
    return nullptr;
  }
  return make_unique<SensorParams>(params);
}

void SensorParams::fillCosSin() 
{
  _row_angles_sines.clear();
  _row_angles_cosines.clear();
  for (const auto& angle : _row_angles) {
    _row_angles_sines.push_back(sin(angle.val()));
    _row_angles_cosines.push_back(cos(angle.val()));
  }
  _col_angles_sines.clear();
  _col_angles_cosines.clear();
  for (const auto& angle : _col_angles) {
    _col_angles_sines.push_back(sin(angle.val()));
    _col_angles_cosines.push_back(cos(angle.val()));
  }
}



}  // namespace cloud_to_image
