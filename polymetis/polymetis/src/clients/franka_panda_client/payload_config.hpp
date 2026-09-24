#pragma once
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace configured_payload {
template<size_t N> bool near(const std::array<double,N>& a,const std::array<double,N>& b,double tol) {
  for(size_t i=0;i<N;++i) if(!std::isfinite(a[i])||!std::isfinite(b[i])||std::abs(a[i]-b[i])>tol) return false;
  return true;
}
inline franka::RobotState apply(franka::Robot& robot,const YAML::Node& cfg) {
  auto state=robot.readOnce();
  if(!cfg || !cfg["enabled"].as<bool>(false)) return state;
  const auto mass=cfg["mass"].as<double>();
  const auto com=cfg["com"].as<std::array<double,3>>();
  const auto inertia=cfg["inertia"].as<std::array<double,9>>();
  if(!std::isfinite(mass)||mass<=0||mass>3.0) throw std::runtime_error("Invalid configured added payload mass");
  for(auto x:com) if(!std::isfinite(x)||std::abs(x)>1.0) throw std::runtime_error("Invalid flange-relative payload COM");
  Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::ColMajor>> I(inertia.data());
  if(!I.allFinite()||(I-I.transpose()).cwiseAbs().maxCoeff()!=0||Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(I).eigenvalues().minCoeff()<=0)
    throw std::runtime_error("Payload inertia must be finite, symmetric and positive definite");
  if(std::abs(state.m_ee-cfg["expected_ee_mass"].as<double>())>1e-5 ||
     !near(state.F_x_Cee,cfg["expected_ee_com"].as<std::array<double,3>>(),1e-5)||
     !near(state.I_ee,cfg["expected_ee_inertia"].as<std::array<double,9>>(),1e-7))
    throw std::runtime_error("Desk end-effector differs from expected stock hand; refusing possible payload double count");
  const bool already=std::abs(state.m_load-mass)<1e-8&&near(state.F_x_Cload,com,1e-8)&&near(state.I_load,inertia,1e-10);
  if(!already && std::abs(state.m_load)>1e-8)
    throw std::runtime_error("An unrelated external payload is already configured; refusing to overwrite it");
  const auto matches = [&]() {
    return std::abs(state.m_load-mass)<=1e-8 && near(state.F_x_Cload,com,1e-8) &&
           near(state.I_load,inertia,1e-10) && std::abs(state.m_total-state.m_ee-mass)<=1e-6;
  };
  if(!already) {
    if(state.robot_mode!=franka::RobotMode::kIdle) throw std::runtime_error("Payload may only be configured while the robot is idle");
    robot.setLoad(mass,com,inertia);
    // Command acknowledgement can precede updated state packets.
    for (int i=0; i<100; ++i) {
      state=robot.readOnce();
      if(matches()) break;
    }
  }
  if(!matches())
    throw std::runtime_error("Payload readback does not match configured added load");
  return state;
}
}
