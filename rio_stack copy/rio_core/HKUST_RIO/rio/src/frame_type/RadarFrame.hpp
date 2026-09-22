#pragma once

#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

#include "eigen3/Eigen/Eigen"
#include "ros/time.h"

namespace Frame {
struct RadarData {
  double azimuth;
  double elevation;
  double range;
  double intensity;
  double doppler;
  float x;
  float y;
  float z;
  double rcs;
  int featureID = -1;

  static inline void anglesToXYZ(RadarData &data) {
    data.x = data.range * cos(data.azimuth) * cos(data.elevation);
    data.y = data.range * sin(data.azimuth) * cos(data.elevation);
    data.z = data.range * sin(data.elevation);
  };

  static inline void xyzToAngles(RadarData &data) {
    data.range = std::sqrt(data.x * data.x + data.y * data.y + data.z * data.z);
    double r_xy = std::sqrt(data.x * data.x + data.y * data.y);
    data.elevation = std::atan2(data.z, r_xy);
    data.azimuth = std::atan2(data.y, data.x);
  }

  static inline void intensityToRCS(RadarData &data) {
    data.rcs = data.intensity * 16 * 3.1415926 * 3.1415926 * data.range *
               data.range * data.range * data.range;
  };

  static inline void rcsToIntensity(RadarData &data) {
    data.intensity = data.rcs / 16 / 3.1415926 / 3.1415926 / data.range /
                     data.range / data.range / data.range;
  }
};

class RadarFrame {
 public:
  RadarFrame() = default;
  ~RadarFrame() = default;
  RadarFrame(RadarFrame &another) = default;
  RadarFrame(RadarFrame &&another) = default;
  RadarFrame &operator=(const RadarFrame &another) = default;
  RadarFrame &operator=(RadarFrame &&another) = default;

  uint32_t id = 0;
  std::vector<RadarData> data;
  std::map<uint32_t, RadarData> matchedPoint;
  std::map<uint32_t, RadarData> unmatchedPoint;
  std::vector<RadarData> staticPoint;
  Eigen::Vector3d gyroData{};
  ros::Time receivedTime = ros::Time(0, 0);
};

class RCSFrame {
 public:
  RCSFrame() = default;
  ~RCSFrame() = default;
  RCSFrame(RCSFrame &another) = default;
  RCSFrame(RCSFrame &&another) = default;
  RCSFrame &operator=(const RCSFrame &another) = default;
  RCSFrame &operator=(RCSFrame &&another) = default;
  uint32_t id = 0;
  ros::Time receivedTime = ros::Time(0, 0);
  std::vector<uint32_t> frameID{};
};

}  // namespace Frame