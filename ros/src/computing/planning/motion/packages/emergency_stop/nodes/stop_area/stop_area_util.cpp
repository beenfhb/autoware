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

#include "stop_area_util.h"

#include <cmath>


// http://www.shibata.nu/kenji/author/cgdam.txt
int lnprm( Point2d pt1, Point2d pt2, double *a, double *b, double *c )
{
    double  xlk, ylk, rsq, rinv;
    double  accy = 1.0E-15;

    xlk = pt2.x - pt1.x;
    ylk = pt2.y - pt1.y;
    rsq = std::pow(xlk, 2.0) + std::pow(ylk, 2.0);

    if(rsq < accy){
        return (-1);
    }else{
        rinv = 1.0 / std::sqrt(rsq);
        *a = -ylk * rinv;
        *b = xlk * rinv;
        *c = (pt1.x * pt2.y - pt2.x * pt1.y) * rinv;
    }

    return (0);
}

int lncl(Point2d pt1, Point2d pt2, Point2d xyr, double r, Point2d pnear, Point2d* xy)
{
    double  root, factor, xo, yo, f, g, fsq, gsq, fgsq, xjo, yjo, a, b, c;
    double  fygx, fxgy, t, fginv, t1, t2, x1, y1, x2, y2, sqdst1, sqdst2;
    double  accy = 1.0E-15;

    if(lnprm(pt1, pt2, &a, &b, &c)) return (-1);

    root = 1.0 / (a*a + b*b);
    factor = -c * root;
    xo = a * factor;
    yo = b * factor;
    root = std::sqrt(root);
    f = b * root;
    g = -a * root;

    fsq = f*f;
    gsq = g*g;
    fgsq = fsq+gsq;

    if(fgsq < accy){
        return (3);
    }else{
        xjo = xyr.x - xo;
        yjo = xyr.y - yo;
        fygx = f*yjo - g*xjo;
        root = r*r*fgsq - fygx*fygx;
        if (root < -accy){
            return (-1);
        }else{
            fxgy = f*xjo + g*yjo;

            if (root < accy){
                t = fxgy / fgsq;
                xy->x = xo + f*t;
                xy->y = yo + g*t;
                return (1);
            }else{
                root = std::sqrt(root);
                fginv = 1.0 / fgsq;
                t1 = (fxgy - root)*fginv;
                t2 = (fxgy + root)*fginv;
                x1 = xo + f*t1;
                y1 = yo + g*t1;
                x2 = xo + f*t2;
                y2 = yo + g*t2;
            }
        }
    }

    sqdst1 = std::pow((pnear.x - x1), 2.0) + std::pow((pnear.y - y1), 2.0);
    sqdst2 = std::pow((pnear.x - x2), 2.0) + std::pow((pnear.y - y2), 2.0);

    if (sqdst1 < sqdst2){
        xy->x = x1;
        xy->y = y1;
    }else{
        xy->x = x2;
        xy->y = y2;
    }

    return (2);
}

Point2d computeIntersectionPoint(const Point2d& pointA, const Point2d& pointB,
							     const Point2d& pointC, const Point2d& pointD)
{
	double dBunbo	= ( pointB.x - pointA.x ) * ( pointD.y - pointC.y )
					- ( pointB.y - pointA.y ) * ( pointD.x - pointC.x );

	if( 0 == dBunbo ) {
		return Point2d();
	}

	Point2d vectorAC;
    vectorAC.x = pointC.x - pointA.x;
    vectorAC.y = pointC.y - pointA.y;

	double dR = ( ( pointD.y - pointC.y ) * vectorAC.x
		 - ( pointD.x - pointC.x ) * vectorAC.y ) / dBunbo;
	double dS = ( ( pointB.y - pointA.y ) * vectorAC.x
		 - ( pointB.x - pointA.x ) * vectorAC.y ) / dBunbo;

    if(dR < 0 || dR > 1.0 || dS < 0 || dS > 1.0) {
        return Point2d();
    }

    Point2d intersection_point;
    intersection_point.x = dR * (pointB.x - pointA.x) + pointA.x;
    intersection_point.y = dR * (pointB.y - pointA.y) + pointA.y;

	return intersection_point;
}

double calcTan(const Point2d& point, const Point2d& con1, const Point2d& con2){
    double Ax,Ay,Bx,By;
    double AxB,AvB;
    double angle;
    Ax = con1.x - point.x;
    Ay = con1.y - point.y;
    Bx = con2.x - point.x;
    By = con2.y - point.y;
    AvB = Ax * Bx + Ay * By;
    AxB = Ax * By - Ay * Bx;

    angle = std::atan2(AxB,AvB);
    return(angle);
}

bool isInPolygon(const Point2d& point, const std::vector<Point2d>& polygon)
{
    double tmp=0;
    double angle=0;

    for(size_t i = 0; i < polygon.size(); i++){
        Point2d con1 = polygon.at(i);
        Point2d con2 = i == polygon.size()-1 ? polygon.at(0) : polygon.at(i+1);
        tmp = calcTan(point, con1, con2);
        angle += tmp;
    }
    if( std::fabs(2.0*M_PI-std::fabs(angle)) < 0.001 ) {
        return true;
    }
    else {
        return false;
    }
}

bool isInPolygon(const Point2d& point, const std::array<Point2d, 4>& polygon)
{
    std::vector<Point2d> array;
    for(const auto p : polygon) {
        array.push_back(p);
    }
    return isInPolygon(point, array);
}
