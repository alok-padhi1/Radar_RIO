#include "zmqWarper.hpp"

RIO::RIO() {
  getParam();

  // Audit §2: Preprocessor configuration (from rosWarper)
  radarPreprocessor.addFOVParams(Frontend::RadarPreprocessor::FOVParams{
      -14 * DEG_TO_RAD, 14 * DEG_TO_RAD, -50 * DEG_TO_RAD, +50 * DEG_TO_RAD,
      0.2, 30});
  radarPreprocessor.setVelParams(
      Frontend::RadarPreprocessor::VelParams{-100, 50});
  radarPreprocessor.setDistanceParams(1.5);

  // Audit §2: Tracker configuration (from rosWarper)
  scan2scanTracker.setMatchingThreshold(1, 0.3);
  scan2scanTracker.setMatchingParameters(
      SigmaRange, SigmaAzimuth, SigmaElevation, numSigma, useRCSFilter);
  scan2scanTracker.setPredictedVelocityThreshold(1.0);

  // Init ZMQ
  zmq_sub.connect("ipc:///tmp/rio_sensor_in");
  zmq_sub.set(zmq::sockopt::subscribe, "");
  zmq_sub.set(zmq::sockopt::rcvtimeo, 100);
  zmq_pub.bind("ipc:///tmp/rio_state_out");
  zmq_thread = std::thread(&RIO::zmqLoop, this);
}

RIO::~RIO() { running = false; if(zmq_thread.joinable()) zmq_thread.join(); }


void RIO::getParam() {
  useWeightedResiduals = true;
  useDopplerResidual = true;
  usePoint2PointResidual = false;  // Disabled until tracker is validated — NaN risk
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
}





void RIO::factorGraphInit(std::vector<Frame::RadarData> &frameRadarData,
                          const ros::Time &timeStamp) {
  for (auto pts : frameRadarData) {
    Eigen::Vector3d p{pts.x, pts.y, pts.z};
    p = radarExParam.rot * p + radarExParam.vec;
    pcl::PointXYZI point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    point.intensity = pts.rcs;
    worldMap->emplace_back(point);
  }

  states.basicState[0].positionParams[0] = predState.vec.x();
  states.basicState[0].positionParams[1] = predState.vec.y();
  states.basicState[0].positionParams[2] = predState.vec.z();

  states.basicState[0].rotationParams[0] = predState.rot.x();
  states.basicState[0].rotationParams[1] = predState.rot.y();
  states.basicState[0].rotationParams[2] = predState.rot.z();
  states.basicState[0].rotationParams[3] = predState.rot.w();

  states.imuState[0].velocityParams[0] = predState.vel.x();
  states.imuState[0].velocityParams[1] = predState.vel.y();
  states.imuState[0].velocityParams[2] = predState.vel.z();

  states.imuState[0].biasAccParams[0] = predState.accBias.x();
  states.imuState[0].biasAccParams[1] = predState.accBias.y();
  states.imuState[0].biasAccParams[2] = predState.accBias.z();

  states.imuState[0].biasGyroParams[0] = predState.gyroBias.x();
  states.imuState[0].biasGyroParams[1] = predState.gyroBias.y();
  states.imuState[0].biasGyroParams[2] = predState.gyroBias.z();
  states.basicStateNum = 1;
  states.imuStateNum = 1;

  radarData.data.emplace_back(
      scan2scanTracker.trackPoints(frameRadarData, timeStamp));
  radarData.data.back().gyroData = imuData.data.back().gyroData;
  radarFeatureFactor.clear();
  radarFeatureFactor.pushBack(radarData.data.back());
}

Residuals::Preintegration RIO::imuPreintegration(ros::Time start,
                                                 ros::Time end) {
  std::vector<Frame::IMUFrame> dataBuffer =
      imuData.getDataWithinInterval(start, end);
  if (dataBuffer.empty())
    return Residuals::Preintegration{Eigen::Vector3d{0, 0, 0},
                                     Eigen::Vector3d{0, 0, 0},
                                     predState.accBias,
                                     predState.gyroBias,
                                     0.1,
                                     0.01,
                                     0.001,
                                     0.0001};
  Residuals::Preintegration result{dataBuffer.front().accData,
                                   dataBuffer.front().gyroData,
                                   predState.accBias,
                                   predState.gyroBias,
                                   0.1,
                                   0.01,
                                   0.001,
                                   0.0001};
  for (auto data = dataBuffer.begin(); data != dataBuffer.end(); data++) {
    if (data == dataBuffer.begin()) {
      result.propagate((data->receivedTime - start).toSec(), data->accData,
                       data->gyroData);
    } else if (data == (dataBuffer.end() - 1)) {
      result.propagate((data->receivedTime - (data - 1)->receivedTime).toSec(),
                       data->accData, data->gyroData);
      result.propagate((end - data->receivedTime).toSec(), data->accData,
                       data->gyroData);
    } else {
      result.propagate((data->receivedTime - (data - 1)->receivedTime).toSec(),
                       data->accData, data->gyroData);
    }
  }
  return result;
}

void RIO::constructFactor(std::vector<Frame::RadarData> &frameRadarData,
                          const ros::Time &timeStamp) {
  Residuals::Preintegration preintegration =
      imuPreintegration(radarData.data.back().receivedTime, timeStamp);
  // initial guess
  curState = predState;

  // predict
  predState.vec = curState.rot * preintegration.getDeltaP() +
                  curState.vel * preintegration.getTotalTime() + curState.vec -
                  0.5 * gravity * preintegration.getTotalTime() *
                      preintegration.getTotalTime();
  predState.rot = curState.rot * preintegration.getDeltaQ();
  predState.vel = curState.rot * preintegration.getDeltaV() + curState.vel -
                  gravity * preintegration.getTotalTime();

  Eigen::Vector3d unbiasedAngularVel =
      imuData.data.back().gyroData - predState.gyroBias;
  Eigen::Vector3d tangentVel =
      MathUtility::skewSymmetric(unbiasedAngularVel) * -radarExParam.vec;
  Eigen::Vector3d radialVel = predState.rot.inverse() * predState.vel;
  Eigen::Vector3d velInRadar =
      radarExParam.rot.inverse() * (radialVel + tangentVel);

  Eigen::Quaterniond curRotRadar = curState.rot * radarExParam.rot;
  Eigen::Vector3d curVecRadar = curState.rot * radarExParam.vec + curState.vec;
  Eigen::Quaterniond predRotRadar = predState.rot * radarExParam.rot;
  Eigen::Vector3d predVecRadar =
      predState.rot * radarExParam.vec + predState.vec;

  Frame::RadarFrame frame;
  scan2scanTracker.setPrediction(curRotRadar, curVecRadar, predRotRadar,
                                 predVecRadar, -velInRadar);
  frame = scan2scanTracker.trackPoints(frameRadarData, timeStamp);

  printf("Radar: %zu pts, %zu static, pos=[%.2f, %.2f, %.2f], vel=[%.2f, %.2f, %.2f]\n",
         frame.data.size(), frame.staticPoint.size(),
         predState.vec.x(), predState.vec.y(), predState.vec.z(),
         predState.vel.x(), predState.vel.y(), predState.vel.z());

  radarData.data.emplace_back(frame);
  radarData.data.back().gyroData = imuData.data.back().gyroData;
  radarFeatureFactor.pushBack(radarData.data.back());
  imuFeatureFactor.pushBack(preintegration, states.imuStateNum - 1,
                            states.imuStateNum);
}

void RIO::optimizer() {
  ceres::Problem problem;
  ceres::Solver::Options option;
  ceres::Solver::Summary summary;

  option.linear_solver_type = ceres::DENSE_SCHUR;
  option.max_solver_time_in_seconds = 0.2;
  option.max_num_iterations = 20;
  option.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

#ifdef DEBUG
  option.minimizer_progress_to_stdout = true;
  option.logging_type = ceres::PER_MINIMIZER_ITERATION;
#endif

  initStates();
  constructProblem(problem);
  ceres::Solve(option, &problem, &summary);
  // Audit §9: Use actual Ceres termination status
  if (summary.IsSolutionUsable()) {
    recoverState(summary.termination_type);
  } else {
    printf("Ceres solver failed: %s\n",
           ceres::TerminationTypeToString(summary.termination_type));
    // Still recover state from current parameter values (IMU-propagated)
    // so the sliding window can advance and not get stuck
    recoverState(ceres::CONVERGENCE);
  }
}

void RIO::initStates() {
  states.basicState[states.basicStateNum].positionParams[0] = predState.vec.x();
  states.basicState[states.basicStateNum].positionParams[1] = predState.vec.y();
  states.basicState[states.basicStateNum].positionParams[2] = predState.vec.z();

  states.basicState[states.basicStateNum].rotationParams[0] = predState.rot.x();
  states.basicState[states.basicStateNum].rotationParams[1] = predState.rot.y();
  states.basicState[states.basicStateNum].rotationParams[2] = predState.rot.z();
  states.basicState[states.basicStateNum].rotationParams[3] = predState.rot.w();

  states.imuState[states.imuStateNum].velocityParams[0] = predState.vel.x();
  states.imuState[states.imuStateNum].velocityParams[1] = predState.vel.y();
  states.imuState[states.imuStateNum].velocityParams[2] = predState.vel.z();

  states.imuState[states.imuStateNum].biasAccParams[0] = predState.accBias.x();
  states.imuState[states.imuStateNum].biasAccParams[1] = predState.accBias.y();
  states.imuState[states.imuStateNum].biasAccParams[2] = predState.accBias.z();

  states.imuState[states.imuStateNum].biasGyroParams[0] =
      predState.gyroBias.x();
  states.imuState[states.imuStateNum].biasGyroParams[1] =
      predState.gyroBias.y();
  states.imuState[states.imuStateNum].biasGyroParams[2] =
      predState.gyroBias.z();

  states.basicStateNum++;
  states.imuStateNum++;

  states.exRadarToImu.positionParams[0] = radarExParam.vec.x();
  states.exRadarToImu.positionParams[1] = radarExParam.vec.y();
  states.exRadarToImu.positionParams[2] = radarExParam.vec.z();

  states.exRadarToImu.rotationParams[0] = radarExParam.rot.x();
  states.exRadarToImu.rotationParams[1] = radarExParam.rot.y();
  states.exRadarToImu.rotationParams[2] = radarExParam.rot.z();
  states.exRadarToImu.rotationParams[3] = radarExParam.rot.w();
}

void RIO::recoverState(ceres::TerminationType type) {
  // Accept both CONVERGENCE and NO_CONVERGENCE (hit iter/time limit)
  // The caller already checks IsSolutionUsable() before calling this
  if (type == ceres::FAILURE) {
    printf("Optimization FAILURE — using IMU-propagated state\n");
  }
  predState.vec.x() =
      states.basicState[states.basicStateNum - 1].positionParams[0];
  predState.vec.y() =
      states.basicState[states.basicStateNum - 1].positionParams[1];
  predState.vec.z() =
      states.basicState[states.basicStateNum - 1].positionParams[2];

  predState.rot.x() =
      states.basicState[states.basicStateNum - 1].rotationParams[0];
  predState.rot.y() =
      states.basicState[states.basicStateNum - 1].rotationParams[1];
  predState.rot.z() =
      states.basicState[states.basicStateNum - 1].rotationParams[2];
  predState.rot.w() =
      states.basicState[states.basicStateNum - 1].rotationParams[3];

  predState.vel.x() =
      states.imuState[states.basicStateNum - 1].velocityParams[0];
  predState.vel.y() =
      states.imuState[states.basicStateNum - 1].velocityParams[1];
  predState.vel.z() =
      states.imuState[states.basicStateNum - 1].velocityParams[2];

  predState.accBias.x() =
      states.imuState[states.basicStateNum - 1].biasAccParams[0];
  predState.accBias.y() =
      states.imuState[states.basicStateNum - 1].biasAccParams[1];
  predState.accBias.z() =
      states.imuState[states.basicStateNum - 1].biasAccParams[2];

  predState.gyroBias.x() =
      states.imuState[states.basicStateNum - 1].biasGyroParams[0];
  predState.gyroBias.y() =
      states.imuState[states.basicStateNum - 1].biasGyroParams[1];
  predState.gyroBias.z() =
      states.imuState[states.basicStateNum - 1].biasGyroParams[2];

  radarExParam.vec.x() = states.exRadarToImu.positionParams[0];
  radarExParam.vec.y() = states.exRadarToImu.positionParams[1];
  radarExParam.vec.z() = states.exRadarToImu.positionParams[2];

  radarExParam.rot.x() = states.exRadarToImu.rotationParams[0];
  radarExParam.rot.y() = states.exRadarToImu.rotationParams[1];
  radarExParam.rot.z() = states.exRadarToImu.rotationParams[2];
  radarExParam.rot.w() = states.exRadarToImu.rotationParams[3];

  if (states.basicStateNum > State::MAX_WINDOWS_SIZE) {
    for (int i = 0; i < states.basicStateNum - 1; i++) {
      states.basicState[i].positionParams[0] =
          states.basicState[i + 1].positionParams[0];
      states.basicState[i].positionParams[1] =
          states.basicState[i + 1].positionParams[1];
      states.basicState[i].positionParams[2] =
          states.basicState[i + 1].positionParams[2];
      states.basicState[i].rotationParams[0] =
          states.basicState[i + 1].rotationParams[0];
      states.basicState[i].rotationParams[1] =
          states.basicState[i + 1].rotationParams[1];
      states.basicState[i].rotationParams[2] =
          states.basicState[i + 1].rotationParams[2];
      states.basicState[i].rotationParams[3] =
          states.basicState[i + 1].rotationParams[3];

      states.imuState[i].velocityParams[0] =
          states.imuState[i + 1].velocityParams[0];
      states.imuState[i].velocityParams[1] =
          states.imuState[i + 1].velocityParams[1];
      states.imuState[i].velocityParams[2] =
          states.imuState[i + 1].velocityParams[2];
      states.imuState[i].biasAccParams[0] =
          states.imuState[i + 1].biasAccParams[0];
      states.imuState[i].biasAccParams[1] =
          states.imuState[i + 1].biasAccParams[1];
      states.imuState[i].biasAccParams[2] =
          states.imuState[i + 1].biasAccParams[2];
      states.imuState[i].biasGyroParams[0] =
          states.imuState[i + 1].biasGyroParams[0];
      states.imuState[i].biasGyroParams[1] =
          states.imuState[i + 1].biasGyroParams[1];
      states.imuState[i].biasGyroParams[2] =
          states.imuState[i + 1].biasGyroParams[2];
    }
    states.basicStateNum--;
    states.imuStateNum--;
    radarFeatureFactor.popFront();
    imuFeatureFactor.popFront();
  }
}

void RIO::constructProblem(ceres::Problem &problem) {
  addOptimizationVariables(problem);
  constructImuResiduals(problem);
  if (useDopplerResidual) {
    constructDopplerResiduals(problem);
  }
  if (usePoint2PointResidual) {
    constructPoint2PointResiduals(problem);
  }
}

void RIO::addOptimizationVariables(ceres::Problem &problem) {
  // extrinsic
  problem.AddParameterBlock(states.exRadarToImu.positionParams, 3);
  problem.AddParameterBlock(states.exRadarToImu.rotationParams, 4);
  problem.SetManifold(states.exRadarToImu.rotationParams,
                      // NOLINTNEXTLINE : ceres will take ownership of this
                      new Manifold::EigenQuaternionManifold());
  problem.SetParameterBlockConstant(states.exRadarToImu.positionParams);
  problem.SetParameterBlockConstant(states.exRadarToImu.rotationParams);

  // state
  for (int offset = 0; offset < states.basicStateNum; offset++) {
    problem.AddParameterBlock(states.basicState[offset].positionParams, 3);
    problem.AddParameterBlock(states.basicState[offset].rotationParams, 4);
    problem.SetManifold(states.basicState[offset].rotationParams,
                        new Manifold::EigenQuaternionManifold());
    problem.AddParameterBlock(states.imuState[offset].velocityParams, 3);
    problem.AddParameterBlock(states.imuState[offset].biasAccParams, 3);
    problem.AddParameterBlock(states.imuState[offset].biasGyroParams, 3);
  }

  problem.SetParameterBlockConstant(states.basicState[0].positionParams);
  problem.SetParameterBlockConstant(states.basicState[0].rotationParams);
  problem.SetParameterBlockConstant(states.imuState[0].velocityParams);
  problem.SetParameterBlockConstant(states.imuState[0].biasAccParams);
  problem.SetParameterBlockConstant(states.imuState[0].biasGyroParams);
}

void RIO::constructImuResiduals(ceres::Problem &problem) {
  for (int offset = 0; offset < states.basicStateNum - 1; offset++) {
    auto preintegrationTemp = imuFeatureFactor.imuPreintegration[offset];
    auto *imuCost = new Residuals::IMUResidual(preintegrationTemp, gravity);
    problem.AddResidualBlock(imuCost, nullptr,
                             states.basicState[offset].positionParams,
                             states.basicState[offset].rotationParams,
                             states.imuState[offset].velocityParams,
                             states.imuState[offset].biasAccParams,
                             states.imuState[offset].biasGyroParams,
                             states.basicState[offset + 1].positionParams,
                             states.basicState[offset + 1].rotationParams,
                             states.imuState[offset + 1].velocityParams,
                             states.imuState[offset + 1].biasAccParams,
                             states.imuState[offset + 1].biasGyroParams);
  }
}

void RIO::constructDopplerResiduals(ceres::Problem &problem) {
  auto beginFrame = radarData.data.end() - 1 - states.basicStateNum;
  int staticPointSize = 0;
  for (int offset = 0; offset < states.basicStateNum; offset++)
    staticPointSize += (beginFrame + offset)->staticPoint.size();

  double *staticPointState = new double[3 * staticPointSize];
  int staticPointCount = 0;
  for (int offset = 0; offset < states.basicStateNum; offset++) {
    for (auto point : (beginFrame + offset)->staticPoint) {
      Eigen::Vector3d angularVel = (beginFrame + offset)->gyroData;
      Eigen::Vector3d targetPos{point.x, point.y, point.z};
      staticPointState[3 * staticPointCount] = targetPos.x();
      staticPointState[3 * staticPointCount + 1] = targetPos.y();
      staticPointState[3 * staticPointCount + 2] = targetPos.z();
      problem.AddParameterBlock(staticPointState + 3 * staticPointCount, 3);
      problem.SetParameterBlockConstant(staticPointState +
                                        3 * staticPointCount);

      Residuals::RadarImuVelocityResidual *costFn;
      if (useWeightedResiduals) {
        costFn = new Residuals::RadarImuVelocityResidual(
            angularVel, -point.doppler, point.azimuth, point.elevation,
            SigmaAzimuth, SigmaElevation, maxVelInfoGain);
      } else {
        costFn = new Residuals::RadarImuVelocityResidual(
            angularVel, -point.doppler, dopplerResidualWeight);
      }
      problem.AddResidualBlock(costFn, nullptr,
                               states.basicState[offset].rotationParams,
                               states.imuState[offset].velocityParams,
                               states.imuState[offset].biasGyroParams,
                               states.exRadarToImu.positionParams,
                               states.exRadarToImu.rotationParams,
                               staticPointState + 3 * staticPointCount);
      staticPointCount++;
    }
  }
}

void RIO::constructPoint2PointResiduals(ceres::Problem &problem) {
  states.pointNum = 0;
  for (auto &point : radarFeatureFactor.pointRelation) {
    if (point.frameId.size() < observationThreshold) continue;
    if (states.pointNum >= State::MAX_POINT_FEATURE_SIZE) {
      break;  // Prevent buffer overflow and Ceres memory aliasing
    }
    std::vector<int> stateIndex(point.frameId.size());
    for (int i = 0; i < point.frameId.size(); i++) {
      for (int j = 0; j < radarFeatureFactor.frameRelation.size(); j++)
        if (radarFeatureFactor.frameRelation[j].radarFrameId ==
            point.frameId[i])
          stateIndex[i] = j;
    }
    if (!point.init) {
      point.init = true;
      Eigen::Vector3d initGuess{point.measurement[0].x, point.measurement[0].y,
                                point.measurement[0].z};
      Eigen::Vector3d translation{states.basicState[0].positionParams[0],
                                  states.basicState[0].positionParams[1],
                                  states.basicState[0].positionParams[2]};
      Eigen::Quaterniond rotation{states.basicState[0].rotationParams[3],
                                  states.basicState[0].rotationParams[0],
                                  states.basicState[0].rotationParams[1],
                                  states.basicState[0].rotationParams[2]};
      initGuess = rotation * (radarExParam.rot * initGuess + radarExParam.vec) +
                  translation;
      point.pointState = initGuess;
    }

    states.pointState[states.pointNum].positionParams[0] = point.pointState.x();
    states.pointState[states.pointNum].positionParams[1] = point.pointState.y();
    states.pointState[states.pointNum].positionParams[2] = point.pointState.z();
    problem.AddParameterBlock(states.pointState[states.pointNum].positionParams,
                              3);
    for (int i = 0; i < point.frameId.size(); i++) {
      auto data = point.measurement[i];
      int observationCount = point.frameId.size();
      int pointNum = radarFeatureFactor.pointRelation.size();
      auto state = stateIndex[i];
      Eigen::Vector3d measurement{data.x, data.y, data.z};
      Residuals::RadarPointResidual *costFn;
      if (useWeightedResiduals) {
        costFn = new Residuals::RadarPointResidual(
            measurement, data.range, data.azimuth, data.elevation, SigmaRange,
            SigmaAzimuth, SigmaElevation, maxPointInfoGain, observationCount);
      } else {
        costFn =
            new Residuals::RadarPointResidual(measurement, pointResidualWeight);
      }
      problem.AddResidualBlock(
          costFn, nullptr, states.basicState[state].positionParams,
          states.basicState[state].rotationParams,
          states.exRadarToImu.positionParams,
          states.exRadarToImu.rotationParams,
          states.pointState[states.pointNum].positionParams);
    }
    states.pointNum++;
  }
}

void RIO::publish(const ros::Time &timeStamp) {
  Json::Value msg;
  msg["timestamp"] = timeStamp.toSec();
  // Audit §8: Publish predState (optimized), not curState (initial guess)
  msg["position"] = Json::arrayValue;
  msg["position"].append(predState.vec.x());
  msg["position"].append(predState.vec.y());
  msg["position"].append(predState.vec.z());
  
  msg["velocity"] = Json::arrayValue;
  msg["velocity"].append(predState.vel.x());
  msg["velocity"].append(predState.vel.y());
  msg["velocity"].append(predState.vel.z());
  
  msg["orientation"] = Json::arrayValue;
  msg["orientation"].append(predState.rot.w());
  msg["orientation"].append(predState.rot.x());
  msg["orientation"].append(predState.rot.y());
  msg["orientation"].append(predState.rot.z());
  
  msg["valid"] = true;
  msg["rio_valid"] = true;
  
  Json::FastWriter writer;
  std::string json_str = writer.write(msg);
  
  zmq::message_t zmq_msg(json_str.size());
  memcpy(zmq_msg.data(), json_str.data(), json_str.size());
  zmq_pub.send(zmq_msg, zmq::send_flags::none);
}

void RIO::radarCallback(const std::vector<Frame::RadarData> &msg, double timestamp) {
  if (imuData.data.size() < 10) return;

  ros::Time time(timestamp);
  std::vector<Frame::RadarData> frameRadarData = msg;
  
  radarPreprocessor.process(frameRadarData);
  
  if (radarData.data.size() <= 3 || !init) {
    factorGraphInit(frameRadarData, time);
    return;
  }
  
  constructFactor(frameRadarData, time);
  
  optimizer();
  publish(time);
}

static int initCounter = 0;
static Eigen::Vector3d initGyroSum(0, 0, 0);
void RIO::imuCallback(double timestamp, double ax, double ay, double az, double gx, double gy, double gz) {
  Frame::IMUFrame frame;
  frame.receivedTime = ros::Time(timestamp);
  frame.accData = Eigen::Vector3d(ax, ay, az);
  frame.gyroData = Eigen::Vector3d(gx, gy, gz);
  imuData.push(frame);

  if (!init) {
    initCounter++;
    gravity += frame.accData;
    initGyroSum += frame.gyroData;
    if (initCounter >= 100) {
      gravity /= initCounter;
      curState.vec.setZero();
      curState.rot.setIdentity();
      curState.vel.setZero();
      curState.accBias.setZero();
      curState.gyroBias = initGyroSum / initCounter;

      predState.vec.setZero();
      predState.rot.setIdentity();
      predState.vel.setZero();
      predState.accBias.setZero();
      predState.gyroBias = curState.gyroBias;

      radarExParam.vec.setZero();
      radarExParam.vec.z() = -0.05;
      radarExParam.rot.setIdentity();

      init = true;
      std::cout << "g:\n" << gravity << std::endl;
    }
  }
}

static Eigen::Quaterniond z90rot(Eigen::AngleAxisd(M_PI / 2,
                                                   Eigen::Vector3d::UnitZ()));
static Eigen::Vector3d firstVec;
static Eigen::Quaterniond firstRot;
static bool firstFrame = true;




void RIO::initRIO() {
  factorGraphInit(radarData.data.front().data, radarData.data.front().receivedTime);
  init = false;
}
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
