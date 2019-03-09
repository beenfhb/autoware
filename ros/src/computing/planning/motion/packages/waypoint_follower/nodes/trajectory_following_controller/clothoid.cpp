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

#include <trajectory_following_controller/clothoid.h>

Clothoid::Clothoid()
{

}

Clothoid::~Clothoid()
{

}

/*
h : length of the Clothoid
phi0 : start angle
phiV : the incremental of the angle if the curventure was not chenged
phiU : the incremental of the angle
*/
void Clothoid::draw(double h,double phi0,double phiV,double phiU)
{
    int n = 1000;
    float stepS = 1.0f / n;
    std::vector<geometry_msgs::Point> points;
    std::complex<float> P_Vector;
    Slope slope(phi0,phiV,phiU);
    for(int i = 0 ; i < n ; ++i)
    {
        float S = stepS * i;
        std::complex<float> r;
        simpsonIntegral(slope, S, S + stepS, &r);
        P_Vector += r;
        float x = P_Vector.real();
        float y = P_Vector.imag();
    }
}

template <class T, class Real, class R>
void Clothoid::simpsonIntegral(T f, Real a, Real b, R *r)
{
    Real mul = (b - a) * static_cast<Real>(1.0 / 6.0);
    *r = mul * (f(a) + static_cast<Real>(4.0) * f((a + b) * static_cast<Real>(0.5)) + f(b));
    return;
}