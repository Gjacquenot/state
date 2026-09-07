#ifndef VESSEL_HPP
#define VESSEL_HPP

#include <gz/math/Matrix3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/System.hh>

#include <ostream>

enum class Convention : int {
    UNKNOWN = 0,
    GAZEBO, //< LOTUSim ROS2 Gazebo, with global ENU_FLU world and body axes
    ENU_FLU, //< East-North-Up world, Forward-Left-Up body
    NED_FRD, //< xdyn
    EUN_FUL, //< Unity3D
    NEU_FRU, //< Unreal 3D
    ERROR
};

struct VesselInformation {
    Convention convention;
    double time;
    gz::sim::Entity entity;
    gz::math::Pose3d pose; //< Position and attitude
    gz::math::Vector3d lin_vel; //< ENU_FLU: world frame. All other conventions: body frame.
    gz::math::Vector3d ang_vel; //< ENU_FLU: world frame. All other conventions: body frame.
    VesselInformation(): convention(Convention::GAZEBO), time(0.0), pose(), lin_vel(), ang_vel(){};
    VesselInformation(Convention conv, double t, const gz::math::Pose3d& p, const gz::math::Vector3d& lv, const gz::math::Vector3d& av):
        convention(conv), time(t), pose(p), lin_vel(lv), ang_vel(av) {};
    VesselInformation to_xdyn() const;   //< enu/flu -> ned/frd
    VesselInformation to_unity() const;  //< enu/flu -> eun/ful
    VesselInformation to_unreal() const; //< enu/flu -> neu/fru
    static VesselInformation from_xdyn(
        const gz::math::Vector3d& xyz,
        const gz::math::Quaterniond& quaternion,
        const gz::math::Vector3d& uvw,
        const gz::math::Vector3d& pqr); //< ned/frd -> enu/flu
};

std::ostream& operator<<(std::ostream& os, Convention s);


void demo();
#endif