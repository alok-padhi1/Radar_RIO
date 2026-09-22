#pragma once

// Std
#include <memory>
#include <string>

// ROS

#include <zmq.hpp>
#include <json/json.h>
#include <thread>
#include <mutex>
#include "ros/time.h"

// Data Manager
#include "IMUManager.hpp"
#include "RadarManager.hpp"
#include "StateManager.hpp"

// Frontend
#include "Preintegration.hpp"
#include "RadarPreprocessor.hpp"
#include "RadarTracker.hpp"

// Factor
#include "ImuPreintegrationManager.hpp"
#include "RadarFeatureManager.hpp"

// Residual
#include "IMUResidual.hpp"
#include "RadarImuVelResidual.hpp"
#include "RadarPointResidual.hpp"

// Utility
#include "MathUtility.hpp"

// PCL Library
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

// Cerse Library
#include <ceres/loss_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/types.h>

// Eigen Library
#include "EigenQuaternionManifold.hpp"

// Const Param
constexpr int SUB_QUEUE_SIZE = 1000;
constexpr int PUB_QUEUE_SIZE = 100;
constexpr float DEG_TO_RAD = 3.1415926 / 180;

// Enum
enum RadarType { ARS548, ColoRadar };

class RIO {
 private:
  // Map
  pcl::PointCloud<pcl::PointXYZI>::Ptr trackingMap =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr worldMap =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  
  // Param
  RadarType radarType;

  bool useWeightedResiduals = true;
  bool useRCSFilter = true;
  bool useDopplerResidual = false;
  bool usePoint2PointResidual = false;

  int observationThreshold = 4;
  double SigmaRange;
  double SigmaAzimuth;
  double SigmaElevation;
  double SigmaDoppler;
  double numSigma;
  double maxVelInfoGain;
  double maxPointInfoGain;
                      double dopplerResidualWeight;
  double pointResidualWeight;

  // ROS
  zmq::context_t zmq_ctx{1};
zmq::socket_t zmq_pub{zmq_ctx, zmq::socket_type::pub};
zmq::socket_t zmq_sub{zmq_ctx, zmq::socket_type::sub};
std::thread zmq_thread;
bool running = true;
void zmqLoop();
        
      
      
  // Data
  Data::IMUManager imuData;
  Data::RadarManager radarData;

  // Frontend
  Frontend::RadarPreprocessor radarPreprocessor;
  Frontend::RadarTracker scan2scanTracker;

  // State
  bool init = false;
  Eigen::Vector3d gravity;
  State::StateManager states;
  struct runtimeState {
    Eigen::Vector3d vec;
    Eigen::Quaterniond rot;
    Eigen::Vector3d vel;
    Eigen::Vector3d accBias;
    Eigen::Vector3d gyroBias;
  };
  runtimeState curState;
  runtimeState predState;

  struct exParam {
    Eigen::Vector3d vec;
    Eigen::Quaterniond rot;
  };
  exParam radarExParam;

  // Factor
  Factor::RadarFeatureManager radarFeatureFactor;
  Factor::ImuPreintegrationManger imuFeatureFactor;

  // BaseFunc
  void getParam();
    void initRIO();
  void publish(const ros::Time &timeStamp);

  // DataProcess
  
  // Estimator
  Residuals::Preintegration imuPreintegration(ros::Time start, ros::Time end);
  void factorGraphInit(std::vector<Frame::RadarData> &frameRadarData,
                       const ros::Time &timeStamp);
  void constructFactor(std::vector<Frame::RadarData> &frameRadarData,
                       const ros::Time &timeStamp);
  void optimizer();
  void initStates();
  void recoverState(ceres::TerminationType type);
  void constructProblem(ceres::Problem &problem);
  void addOptimizationVariables(ceres::Problem &problem);
  void constructImuResiduals(ceres::Problem &problem);
  void constructDopplerResiduals(ceres::Problem &problem);
  void constructPoint2PointResiduals(ceres::Problem &problem);

  // Callback
  void radarCallback(const std::vector<Frame::RadarData> &msg, double timestamp);
  void imuCallback(double timestamp, double ax, double ay, double az, double gx, double gy, double gz);
    
 public:
  RIO();
  ~RIO();

  RIO(RIO &another) = delete;
  RIO(RIO &&another) = delete;
};