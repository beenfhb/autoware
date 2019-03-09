#ifndef WAYPOINT_FOLLWER_SLOPE_H_INCLUDED
#define WAYPOINT_FOLLWER_SLOPE_H_INCLUDED

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

//headers in STL
#include <complex>

class Slope
{
public:
    Slope(float phi0, float phiV, float phiU) : phi0(phi0),phiV(phiV),phiU(phiU){};
    const float phi0;
    const float phiV;
    const float phiU;
    std::complex<float> operator()(float S)
    {
        std::complex<float> j(0.0f, 1.0f);
        return std::exp(j * phi(S));
    }
private:
    float inline phi(float S) {return phi0 + phiV * S + phiU * S * S;};
};

#endif  //WAYPOINT_FOLLWER_SLOPE_H_INCLUDED