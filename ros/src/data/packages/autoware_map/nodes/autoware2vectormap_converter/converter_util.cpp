#include <autoware2vectormap_converter/autoware2vectormap_converter.h>

int convertESPGToRef(int epsg)
{

    if(epsg >= 2443 && epsg <= 2461)
    {
        return epsg - 2442;
    }
    if(epsg >= 6669 && epsg <= 6687)
    {
        return epsg - 6668;
    }
    return 0;
}

//keep angles within (M_PI, -M_PI]
double addAngles(double angle1, double angle2)
{
    double sum = angle1 + angle2;
    while( sum > M_PI ) sum -= 2 * M_PI;
    while( sum <= -M_PI ) sum += 2 * M_PI;
    return sum;
}

double convertDecimalToDDMMSS(const double decimal)
{
    int degree, minutes,seconds;
    degree = floor(decimal);
    minutes = floor( (decimal - degree ) * 60);
    seconds = floor( (decimal - degree - minutes * 1.0 / 60) * 3600);
    return degree + minutes * 0.01 + seconds * 0.0001;
}


void getMinMax(autoware_map_msgs::Point &min, autoware_map_msgs::Point &max, const std::vector<autoware_map_msgs::Point>points)
{
    min = max = points.front();
    for (auto pt : points)
    {
        min.x = min.x < pt.x ? min.x : pt.x;
        min.y = min.y < pt.y ? min.y : pt.y;
        min.z = min.z < pt.z ? min.z : pt.z;
        max.x = max.x > pt.x ? max.x : pt.x;
        max.y = max.y > pt.y ? max.y : pt.y;
        max.z = max.z > pt.z ? max.z : pt.z;
    }
}

//determine whether point lies within an area by counting winding number
bool isWithinArea(double x, double y, const std::vector<autoware_map_msgs::Point> vertices)
{
    std::vector<double> angles;
    for (auto pt : vertices)
    {
        if(pt.x == x && pt.y== y) return false;
        angles.push_back( atan2(pt.y - y, pt.x - x));
    }

    double angle_sum = 0;
    for (unsigned int idx = 0; idx < vertices.size(); idx++)
    {
        double angle1, angle2;
        angle1 = angles.at(idx);
        if(idx + 1 >= vertices.size())
        {
            angle2 = angles.front();
        }else
        {
            angle2 = angles.at(idx + 1);
        }
        double angle_diff = addAngles(angle1, -angle2);
        angle_sum += angle_diff;
    }
    //allow some precision error
    if(fabs(angle_sum) < 1e-3) return false;
    else return true;
}

bool getIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double &intersect_x, double &intersect_y )
{
    //let p1(x1, y1), p2(x2, y2), p3(x3, y3), p4(x4,y4)
    //intersect of line segment p1 to p2 and p3 to p4 satisfies
    // p1 + r(p2 - p1) = p3 + s(p4 - p3)
    // 0 <= r <= 1
    // 0 <= s <= 1
    double denominator = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);

    if(denominator == 0) {
        //line is parallel
        return false;
    }

    double r = ( (y4 - y3) * (x3 - x1) - (x4 - x3) * (y3 - y1) ) / denominator;
    double s = ( (y2 - y1) * (x3 - x1) - (x2 - x1) * (y3 - y1) ) / denominator;

    if( r >= 0 && r <= 1 && s >= 0 && s <= 1) {
        intersect_x = x1 + r * (x2 - x1);
        intersect_y = y1 + r * (y2 - y1);
        return true;
    }else{
        return false;
    }
}

int getMaxId(std::vector<vector_map_msgs::Point> points){
  int max = 0;
  for (auto p: points){
    if (p.pid > max){
      max = p.pid;
    }
  }
  return max;
}

int getMaxId(std::vector<vector_map_msgs::Line> lines){
  int max = 0;
  for (auto l: lines){
    if (l.lid > max){
      max = l.lid;
    }
  }
  return max;
}
int getMaxId(std::vector<vector_map_msgs::StopLine> stop_lines){
  int max = 0;
  for (auto l: stop_lines){
    if (l.id > max){
      max = l.id;
    }
  }
  return max;
}
int getMaxId(std::vector<vector_map_msgs::RoadSign> signs){
  int max = 0;
  for (auto s: signs){
    if (s.id > max){
      max = s.id;
    }
  }
  return max;
}

int getMaxId(std::vector<vector_map_msgs::Area> areas){
  int max = 0;
  for (auto a: areas){
    if (a.aid > max){
      max = a.aid;
    }
  }
  return max;
}
