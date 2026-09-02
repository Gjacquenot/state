#include <iostream>
#include <stdexcept>
#include <gz/sim/System.hh>

enum class Convention : int {
    Unknown = 0,
    ENU_FLU, //< LOTUSim ROS2 Gazebo
    NED_FRD, //< xdyn
    EUN_FUL, //< Unity3D
    NEU_FRU, //< Unreal 3D
    Error
};

std::ostream& operator<<(std::ostream& os, Convention s) {
    switch (s) {
        case Convention::Unknown:  return os << "Unknown";
        case Convention::ENU_FLU:  return os << "ENU_FLU";
        case Convention::NED_FRD:  return os << "NED_FRD";
        case Convention::EUN_FUL:  return os << "EUN_FUL";
        case Convention::NEU_FRU:  return os << "NEU_FRU";
    }
    return os << "Unknown";
}

static const gz::math::Quaterniond q_ned_to_enu(0.0, 0.5 * sqrt(2.0), 0.5 * sqrt(2.0), 0.0);

// Body-frame convention swap (gz FLU <-> xdyn FRD): 180 deg about body-x.
static const gz::math::Quaterniond q_flu_to_frd(0.0, 1.0, 0.0, 0.0);

/// Body basis change, xdyn FRD (x forward, y starboard, z down) to Gazebo FLU
/// (x forward, y port, z up): 180 deg about body X. Distinct from q_ned_to_enu.
static const gz::math::Quaterniond q_frd_to_flu(0.0, 1.0, 0.0, 0.0);


// Convert a quaternion from the NED frame to the ENU frame

// An attitude quaternion maps body axes to world axes, so converting it
// between conventions changes BOTH frames: world swap (ENU<->NED) on the
// left, body swap (FLU<->FRD) on the right. A similarity transform
// (q_swap * q * q_swap^-1) only relabels the world frame and yields
// yaw_ned = -yaw_enu instead of the correct yaw_ned = pi/2 - yaw_enu.
// Both swap factors are 180-deg rotations (involutive up to sign), so the
// same product converts in either direction.
gz::math::Quaterniond quatNedToEnu(const gz::math::Quaterniond& q_ned)
{
    return q_ned_to_enu * q_ned * q_flu_to_frd;
}

// Convert a quaternion from the ENU frame back to the NED frame.
gz::math::Quaterniond quatEnuToNed(const gz::math::Quaterniond& q_enu)
{
    return q_ned_to_enu * q_enu * q_flu_to_frd;
}

gz::math::Vector3d vecNedToEnuFixedFrame(const gz::math::Vector3d& v_ned)
{
    return {v_ned.Y(), v_ned.X(), -v_ned.Z()};
}

gz::math::Vector3d vecNedToEnuBodyFrame(const gz::math::Vector3d& v_ned)
{
    return {v_ned.X(), -v_ned.Y(), -v_ned.Z()};
}

gz::math::Vector3d vecEnuToNedFixedFrame(const gz::math::Vector3d& v_enu)
{
    return {v_enu.Y(), v_enu.X(), -v_enu.Z()};
}

gz::math::Vector3d vecEnuToNedBodyFrame(const gz::math::Vector3d& v_enu)
{
    return {v_enu.X(), -v_enu.Y(), -v_enu.Z()};
}

struct VesselInformation {
    Convention convention;
    double time;
    gz::sim::Entity entity;
    gz::math::Pose3d pose; //< Position and attitude in ENU ground frame
    gz::math::Vector3d lin_vel; //< Linear velocity expressed in ENU ground frame
    gz::math::Vector3d ang_vel; //< Angular velocity expressed in ENU ground frame
    VesselInformation(): convention(Convention::ENU_FLU), time(0.0), pose(), lin_vel(), ang_vel(){};
    VesselInformation to_xdyn() const; //< enu/flu -> ned/frd
    VesselInformation to_unity() const; //< enu/flu -> neu/fru
    VesselInformation to_unreal() const; //< enu/flu -> enu/flu
    static VesselInformation from_xdyn(
        const gz::math::Vector3d& xyz,
        const gz::math::Quaterniond& quaternion,
        const gz::math::Vector3d& uvw,
        const gz::math::Vector3d& pqr); //< ned/frd -> enu/flu
};

VesselInformation VesselInformation::to_xdyn() const //< enu/flu -> ned/frd
{
    // TODO
    if (convention != Convention::ENU_FLU)
        throw std::runtime_error("Invalid convention");
    VesselInformation v;
    v.convention = Convention::NED_FRD;
    return v;
}
VesselInformation VesselInformation::to_unity() const //< enu/flu -> neu/fru
{
    // TODO
    VesselInformation v;
    v.convention = Convention::NEU_FRU;
    return v;
}

VesselInformation VesselInformation::to_unreal() const //< enu/flu -> enu/flu
{
    // TODO
    VesselInformation v;
    v.convention = Convention::NEU_FRU;
    return v;
}

VesselInformation VesselInformation::from_xdyn(
        const gz::math::Vector3d& ned_xyz,
        const gz::math::Quaterniond& ned_quaternion,
        const gz::math::Vector3d& ned_uvw,
        const gz::math::Vector3d& ned_pqr)
{
    // TODO
    VesselInformation s;
    const gz::math::Vector3d enu_xyz = vecNedToEnuFixedFrame(ned_xyz);
    const gz::math::Quaterniond enu_quaternion = quatNedToEnu(ned_quaternion);
    s.pose = gz::math::Pose3d(enu_xyz, enu_quaternion);
    return s;
}


int main()
{

    return 0;
}