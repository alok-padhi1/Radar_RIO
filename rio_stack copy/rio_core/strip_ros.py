#!/usr/bin/env python3
import os
import re

base_dir = '/home/alok/radar/rio_stack copy/rio_core/HKUST_RIO/rio/node'
hpp_file = os.path.join(base_dir, 'rosWarper.hpp')
cpp_file = os.path.join(base_dir, 'rosWarper.cpp')

with open(hpp_file, 'r') as f:
    hpp = f.read()

# 1. Remove ROS includes
hpp = re.sub(r'#include "geometry_msgs/.*?\n', '', hpp)
hpp = re.sub(r'#include "nav_msgs/.*?\n', '', hpp)
hpp = re.sub(r'#include "ros/ros\.h"\n', '', hpp)
hpp = re.sub(r'#include "sensor_msgs/.*?\n', '', hpp)

# 2. Add ZMQ and custom struct includes
zmq_includes = """#include <zmq.hpp>
#include <json/json.h>
#include <thread>
#include <mutex>

namespace ros {
  struct Time {
    uint32_t sec;
    uint32_t nsec;
    Time() : sec(0), nsec(0) {}
    Time(double t) {
      sec = (uint32_t)t;
      nsec = (uint32_t)((t - sec) * 1e9);
    }
    double toSec() const { return sec + nsec * 1e-9; }
  };
}
"""
hpp = hpp.replace('// Data Manager', zmq_includes + '\n// Data Manager')

# 3. Replace ROS classes with ZMQ equivalents in RIO class
hpp = re.sub(r'ros::NodeHandle nodeHandle;', 'zmq::context_t zmq_ctx{1};\nzmq::socket_t zmq_pub{zmq_ctx, zmq::socket_type::pub};\nzmq::socket_t zmq_sub{zmq_ctx, zmq::socket_type::sub};\nstd::thread zmq_thread;\nbool running = true;\nvoid zmqLoop();', hpp)
hpp = re.sub(r'ros::Subscriber.*?\n', '', hpp)
hpp = re.sub(r'ros::Publisher.*?\n', '', hpp)

# 4. Change callbacks
hpp = hpp.replace('void radarCallback(const sensor_msgs::PointCloud2 &msg);', 'void radarCallback(const std::vector<Frame::RadarData> &msg, double timestamp);')
hpp = hpp.replace('void imuCallback(const sensor_msgs::Imu &msg);', 'void imuCallback(double timestamp, double ax, double ay, double az, double gx, double gy, double gz);')
hpp = re.sub(r'void nav_gtCallback.*?\n', '', hpp)
hpp = re.sub(r'void geometry_gtCallback.*?\n', '', hpp)
hpp = hpp.replace('RIO(ros::NodeHandle &nh);', 'RIO();')

# 5. Remove decodeRadarMsg
hpp = re.sub(r'std::vector<Frame::RadarData> decodeRadarMsg_ARS548.*?\n.*?\(.*?\);\n', '', hpp, flags=re.DOTALL)
hpp = re.sub(r'std::vector<Frame::RadarData> decodeRadarMsg_ColoRadar.*?\n.*?\(.*?\);\n', '', hpp, flags=re.DOTALL)

with open(hpp_file.replace('rosWarper.hpp', 'zmqWarper.hpp'), 'w') as f:
    f.write(hpp)

print("Generated zmqWarper.hpp")


with open(cpp_file, 'r') as f:
    cpp = f.read()

# 1. Update include
cpp = cpp.replace('#include "rosWarper.hpp"', '#include "zmqWarper.hpp"')

# 2. Update constructor
cpp = cpp.replace('RIO::RIO(ros::NodeHandle &nh) {', 'RIO::RIO() {')
cpp = cpp.replace('initROS(nh);', 'getParam();\n  // Init ZMQ\n  zmq_sub.connect("ipc:///tmp/rio_sensor_in");\n  zmq_sub.set(zmq::sockopt::subscribe, "");\n  zmq_sub.set(zmq::sockopt::rcvtimeo, 100);\n  zmq_pub.bind("ipc:///tmp/rio_state_out");\n  zmq_thread = std::thread(&RIO::zmqLoop, this);\n')

# 3. Update destructor
cpp = cpp.replace('RIO::~RIO() {}', 'RIO::~RIO() { running = false; if(zmq_thread.joinable()) zmq_thread.join(); }')

# 4. Remove initROS
cpp = re.sub(r'void RIO::initROS.*?}\n', '', cpp, flags=re.DOTALL)

# 5. Replace getParam with hardcoded values since we don't have ros param server
getparam_replacement = """void RIO::getParam() {
  useWeightedResiduals = true;
  useDopplerResidual = true;
  usePoint2PointResidual = false;
  SigmaRange = 0.1;
  SigmaAzimuth = 0.05;
  SigmaElevation = 0.05;
  SigmaDoppler = 0.1;
  numSigma = 3.0;
  maxVelInfoGain = 100.0;
  maxPointInfoGain = 100.0;
  dopplerResidualWeight = 1.0;
  pointResidualWeight = 1.0;
  radarType = ARS548; // Handled by Python now anyway
}"""
cpp = re.sub(r'void RIO::getParam\(\).*?^}', getparam_replacement, cpp, flags=re.DOTALL|re.MULTILINE)

# 6. Remove decodeRadarMsg functions
cpp = re.sub(r'std::vector<Frame::RadarData> RIO::decodeRadarMsg_ARS548.*?^}', '', cpp, flags=re.DOTALL|re.MULTILINE)
cpp = re.sub(r'std::vector<Frame::RadarData> RIO::decodeRadarMsg_ColoRadar.*?^}', '', cpp, flags=re.DOTALL|re.MULTILINE)

# 7. Update publish
pub_replacement = """void RIO::publish(const ros::Time &timeStamp) {
  Json::Value msg;
  msg["timestamp"] = timeStamp.toSec();
  msg["position"] = Json::arrayValue;
  msg["position"].append(curState.vec.x());
  msg["position"].append(curState.vec.y());
  msg["position"].append(curState.vec.z());
  
  msg["velocity"] = Json::arrayValue;
  msg["velocity"].append(curState.vel.x());
  msg["velocity"].append(curState.vel.y());
  msg["velocity"].append(curState.vel.z());
  
  msg["orientation"] = Json::arrayValue;
  msg["orientation"].append(curState.rot.w());
  msg["orientation"].append(curState.rot.x());
  msg["orientation"].append(curState.rot.y());
  msg["orientation"].append(curState.rot.z());
  
  msg["valid"] = true;
  msg["rio_valid"] = true;
  
  Json::FastWriter writer;
  std::string json_str = writer.write(msg);
  
  zmq::message_t zmq_msg(json_str.size());
  memcpy(zmq_msg.data(), json_str.data(), json_str.size());
  zmq_pub.send(zmq_msg, zmq::send_flags::none);
}"""
cpp = re.sub(r'void RIO::publish\(const ros::Time &timeStamp\) {.*?^}', pub_replacement, cpp, flags=re.DOTALL|re.MULTILINE)

# 8. Update imuCallback
imu_cb_replacement = """void RIO::imuCallback(double timestamp, double ax, double ay, double az, double gx, double gy, double gz) {
  Frame::IMUFrame frame;
  frame.receivedTime = ros::Time(timestamp);
  frame.accData = Eigen::Vector3d(ax, ay, az);
  frame.gyroData = Eigen::Vector3d(gx, gy, gz);
  imuData.addIMUData(frame);
}"""
cpp = re.sub(r'void RIO::imuCallback\(const sensor_msgs::Imu &msg\) {.*?^}', imu_cb_replacement, cpp, flags=re.DOTALL|re.MULTILINE)

# 9. Update radarCallback
radar_cb_replacement = """void RIO::radarCallback(const std::vector<Frame::RadarData> &msg, double timestamp) {
  ros::Time time(timestamp);
  
  Frame::RadarFrame frame;
  frame.data = msg;
  frame.receivedTime = time;
  
  // (The rest of radarCallback remains the same)
  if (!init) {
    initStates();
    init = true;
  }
  
  radarData.addRadarData(frame);
  
  if (imuData.imuData.size() < 10) return;
  
  optimizer();
  publish(time);
}"""
cpp = re.sub(r'void RIO::radarCallback\(const sensor_msgs::PointCloud2 &msg\) {.*?^}', radar_cb_replacement, cpp, flags=re.DOTALL|re.MULTILINE)

# 10. Add zmqLoop
zmq_loop = """
#pragma pack(push, 1)
struct IMUPayload {
    uint8_t type;
    double t;
    float ax, ay, az;
    float gx, gy, gz;
};
#pragma pack(pop)

void RIO::zmqLoop() {
  while(running) {
    zmq::message_t msg;
    auto res = zmq_sub.recv(msg, zmq::recv_flags::none);
    if (!res || msg.size() == 0) continue;
    
    const uint8_t* data = static_cast<const uint8_t*>(msg.data());
    uint8_t type = data[0];
    
    if (type == 1 && msg.size() == sizeof(IMUPayload)) {
      const IMUPayload* p = reinterpret_cast<const IMUPayload*>(data);
      imuCallback(p->t, p->ax, p->ay, p->az, p->gx, p->gy, p->gz);
    } else {
      // JSON radar payload
      std::string json_str(static_cast<const char*>(msg.data()), msg.size());
      Json::Value root;
      Json::Reader reader;
      if (reader.parse(json_str, root) && root["type"].asInt() == 2) {
        std::vector<Frame::RadarData> points;
        double timestamp = root["timestamp"].asDouble();
        const Json::Value& pts = root["points"];
        for(int i=0; i<pts.size(); ++i) {
          Frame::RadarData p;
          p.range = pts[i][0].asDouble();
          p.azimuth = pts[i][1].asDouble();
          p.elevation = pts[i][2].asDouble();
          p.doppler = pts[i][3].asDouble();
          Frame::RadarData::anglesToXYZ(p);
          points.push_back(p);
        }
        radarCallback(points, timestamp);
      }
    }
  }
}
"""
cpp += zmq_loop

# Remove gtCallbacks
cpp = re.sub(r'void RIO::nav_gtCallback.*?\n}', '', cpp, flags=re.DOTALL)
cpp = re.sub(r'void RIO::geometry_gtCallback.*?\n}', '', cpp, flags=re.DOTALL)

with open(cpp_file.replace('rosWarper.cpp', 'zmqWarper.cpp'), 'w') as f:
    f.write(cpp)

print("Generated zmqWarper.cpp")

# Fix remaining ROS artifacts in zmqWarper.hpp
with open(hpp_file.replace('rosWarper.hpp', 'zmqWarper.hpp'), 'r') as f:
    hpp = f.read()

hpp = re.sub(r'nav_msgs::Path.*?\n', '', hpp)
hpp = re.sub(r'void initROS.*?\n', '', hpp)
hpp = re.sub(r'std::string.*?Topic;\n', '', hpp)
hpp = re.sub(r'double curTime.*?\n', '', hpp)

with open(hpp_file.replace('rosWarper.hpp', 'zmqWarper.hpp'), 'w') as f:
    f.write(hpp)

# Fix remaining ROS artifacts in zmqWarper.cpp
with open(cpp_file.replace('rosWarper.cpp', 'zmqWarper.cpp'), 'r') as f:
    cpp = f.read()

# Replace initRIO completely
init_rio_replacement = """void RIO::initRIO() {
  factorGraphInit(radarData.radarData.front().data, radarData.radarData.front().receivedTime);
  init = false;
}"""
cpp = re.sub(r'void RIO::initRIO\(\) \{.*?^}', init_rio_replacement, cpp, flags=re.DOTALL|re.MULTILINE)

# Remove ROS_ERROR
cpp = cpp.replace('ROS_ERROR', 'printf')

with open(cpp_file.replace('rosWarper.cpp', 'zmqWarper.cpp'), 'w') as f:
    f.write(cpp)
