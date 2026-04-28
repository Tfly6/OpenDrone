#ifndef POINT_STRUCT_H
#define POINT_STRUCT_H

#include <math.h>
#include <vector>
#include <utility>
#include <iostream>
#include <stdlib.h>
#include <deque>
#include <unordered_map>
#include "Eigen/Core"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/functional/hash.hpp>

// ROS message support
#include <ros/ros.h>


typedef pcl::PointXYZI PCLPoint;
typedef pcl::PointCloud<PCLPoint> PointCloud;
typedef pcl::PointCloud<PCLPoint>::Ptr PointCloudPtr;

struct Point3D {
  float x, y, z;
  float intensity;
  Point3D() = default;
  Point3D(float _x, float _y, float _z):\
      x(_x), y(_y), z(_z), intensity(0) {}
  Point3D(float _x, float _y, float _z, float _i):\
      x(_x), y(_y), z(_z), intensity(_i) {}
  Point3D(Eigen::Vector3f vec):\
      x(vec(0)), y(vec(1)), z(vec(2)), intensity(0) {}
  Point3D(Eigen::Vector3d vec):\
      x(vec(0)), y(vec(1)), z(vec(2)), intensity(0) {}
  Point3D(PCLPoint pt):\
      x(pt.x), y(pt.y), z(pt.z), intensity(pt.intensity) {}

  bool operator ==(const Point3D& pt) const
  {
      return x == pt.x && y == pt.y && z == pt.z;
  }

  bool operator !=(const Point3D& pt) const
  {
    return x != pt.x || y != pt.y || z != pt.z;
  }

  float operator *(const Point3D& pt) const
  {
      return x * pt.x + y * pt.y + z * pt.z;
  }

  Point3D operator *(const float factor) const
  {
      return Point3D(x*factor, y*factor, z*factor);
  }

  Point3D operator /(const float factor) const
  {
      return Point3D(x/factor, y/factor, z/factor);
  }

  Point3D operator +(const Point3D& pt) const
  {
      return Point3D(x+pt.x, y+pt.y, z+pt.z);
  }
  template <typename Point>
  Point3D operator -(const Point& pt) const
  {
      return Point3D(x-pt.x, y-pt.y, z-pt.z);
  }
  Point3D operator -() const
  {
      return Point3D(-x, -y, -z);
  }
  float norm() const
  {
    return std::hypotf(x, std::hypotf(y,z));
  };

  Point3D normalize() const 
  {
    const float n = std::hypotf(x, std::hypotf(y,z));
    if (n > 1e-5) {
      return Point3D(x/n, y/n, z/n);
    } else {
      ROS_ERROR("vector normalization fails, vector norm is too small.");
      return Point3D(0,0,0);
    }
  };

  float norm_dot(Point3D p) const
  {
    const float n1 = std::hypotf(x, std::hypotf(y,z));
    const float n2 = std::hypotf(p.x, std::hypotf(p.y,p.z));
    if (n1 < 1e-5 || n2 < 1e-5) {
      ROS_ERROR("vector norm dot fails, vector norm is too small.");
    }
    const float dot_value = (x * p.x + y * p.y + z * p.z) / (n1 * n2);
    return std::min(std::max((float)-1.0, dot_value), (float)1.0);
  };

  std::string ToString() const
  {
    return "x = " + std::to_string(x).substr(0,6) + "; " + 
           "y = " + std::to_string(y).substr(0,6) + "; " + 
           "z = " + std::to_string(z).substr(0,6) + " "; 
  };
  
  friend std::ostream& operator<<(std::ostream& os, const Point3D& p)
  {
    os << "["<< p.ToString() << "] ";
    return os;
  }
};

struct point_hash
{
  std::size_t operator() (const Point3D& p) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, p.x);
    boost::hash_combine(seed, p.y);
    boost::hash_combine(seed, p.z);
    return seed;
  }
};

struct point_comp
{
  bool operator()(const Point3D& p1, const Point3D& p2) const
  {
    return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z;
  }
};

struct intensity_comp
{
  bool operator()(const Point3D& p1, const Point3D& p2) const
  {
    return p1.intensity < p2.intensity;
  }
};

#endif