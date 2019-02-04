/*
 *  Copyright (c) 2018, TierIV, Inc
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 #ifndef STOP_AREA_UTIL_H
 #define STOP_AREA_UTIL_H

#include <vector>
#include <array>

 struct Point2d{
     Point2d() : x(0), y(0){};
     Point2d(double x, double y) : x(x), y(y){};
     double x;
     double y;
 };

 int lnprm( Point2d pt1, Point2d pt2, double *a, double *b, double *c );
 int lncl(Point2d pt1, Point2d pt2, Point2d xyr, double r, Point2d pnear, Point2d* xy);
 Point2d computeIntersectionPoint(const Point2d& pointA, const Point2d& pointB,
 							     const Point2d& pointC, const Point2d& pointD);
 double Calc_tan(const Point2d& point, const Point2d& con1, const Point2d& con2);
 bool isInPolygon(const Point2d& point, const std::vector<Point2d>& polygon);
 bool isInPolygon(const Point2d& point, const std::array<Point2d, 4>& polygon);

#endif
