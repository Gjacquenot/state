#include "vessel.hpp"

#include <iostream>
#include <stdexcept>

/**
 * @brief Stream a human-readable coordinate convention name.
 */
std::ostream& operator<<(std::ostream& os, Convention s)
{
    switch (s) {
        case Convention::UNKNOWN:  return os << "Unknown";
        case Convention::GAZEBO:   return os << "GAZEBO";
        case Convention::ENU_FLU:  return os << "ENU_FLU";
        case Convention::NED_FRD:  return os << "NED_FRD";
        case Convention::EUN_FUL:  return os << "EUN_FUL";
        case Convention::NEU_FRU:  return os << "NEU_FRU";
    }
    return os << "Unknown";
}

// ---------------------------------------------------------------------------
// Axis relabeling matrices, all expressed as "v_target = C * v_reference",
// with ENU_FLU (world East-North-Up, body Forward-Left-Up) as the reference.
//
// Every matrix below is an involution (C*C = I / self-inverse), so the SAME
// matrix converts in both directions -- there's no separate "forward" and
// "reverse" version of any of these.
//
// IMPORTANT: for a given target, the world matrix and body matrix are NOT
// always the same operation. EUN_FUL happens to use the identical Y<->Z
// swap for both (Unity's body axes mirror its world axes labeling), but
// NEU_FRU does not: Unreal's real body convention is Forward-Right-Up,
// which is a Left<->Right flip relative to FLU, while its world convention
// (North-East-Up) is an X<->Y swap relative to ENU. Those are genuinely
// different matrices with no simple relationship to each other, so the
// quaternion for that pair can't be built from two multiplied constants --
// it has to go through the rotation matrix (see quatChangeFrame below).
// ---------------------------------------------------------------------------

// World: ENU (East,North,Up) <-> NED (North,East,Down): x_ned=y_enu, y_ned=x_enu, z_ned=-z_enu
static const gz::math::Matrix3d kWorldEnuNed(0, 1, 0,  1, 0, 0,  0, 0, -1);
// Body: FLU (Fwd,Left,Up) <-> FRD (Fwd,Right,Down): u_frd=u_flu, v_frd=-v_flu, w_frd=-w_flu
static const gz::math::Matrix3d kBodyFluFrd(1, 0, 0,  0, -1, 0,  0, 0, -1);

// World: ENU (East,North,Up) <-> EUN (East,Up,North): swap Y,Z (Unity)
static const gz::math::Matrix3d kWorldEnuEun(1, 0, 0,  0, 0, 1,  0, 1, 0);
// Body: FLU (Fwd,Left,Up) <-> FUL (Fwd,Up,Left): swap Y,Z -- mirrors the
// world swap above for this convention.
static const gz::math::Matrix3d kBodyFluFul(1, 0, 0,  0, 0, 1,  0, 1, 0);

// World: ENU (East,North,Up) <-> NEU (North,East,Up): swap X,Y (Unreal)
static const gz::math::Matrix3d kWorldEnuNeu(0, 1, 0,  1, 0, 0,  0, 0, 1);
// Body: FLU (Fwd,Left,Up) <-> FRU (Fwd,Right,Up): Left<->Right flip only.
// NOTE this deliberately does NOT match kWorldEnuNeu's X,Y swap -- see the
// block comment above.
static const gz::math::Matrix3d kBodyFluFru(1, 0, 0,  0, -1, 0,  0, 0, 1);

// ---------------------------------------------------------------------------
// Generic, always-correct conversions
// ---------------------------------------------------------------------------

// An attitude quaternion maps body axes to world axes. Converting it to a
// convention with a different world AND body relabeling requires changing
// both sides: v_new = worldC * R * bodyC^-1 * v_ref, at the matrix level.
//
// This is deliberately NOT a similarity transform (q_C * q * q_C^-1) with a
// single q_C: worldC and bodyC are generally different matrices (see
// kBodyFluFru vs kWorldEnuNeu), and individually they are often improper
// (det = -1, a handedness flip) and so don't correspond to any quaternion
// at all -- only their combination is guaranteed to be a proper rotation.
// Going through the rotation matrix sidesteps that issue entirely and is
// correct for every convention pair in this file, including the ones (like
// NED_FRD) where a hand-multiplied quaternion shortcut also happens to
// exist.
/**
 * @brief Change an attitude quaternion between world and body axis frames.
 * @param q Source attitude quaternion.
 * @param worldC World-frame axis conversion matrix.
 * @param bodyC Body-frame axis conversion matrix.
 * @return The attitude quaternion in the target convention.
 */
gz::math::Quaterniond quatChangeFrame(
    const gz::math::Quaterniond& q,
    const gz::math::Matrix3d& worldC,
    const gz::math::Matrix3d& bodyC)
{
    const gz::math::Matrix3d R(q);
    // bodyC is self-inverse for every case in this file; Inverse() is kept
    // explicit here for correctness in case that ever stops being true.
    const gz::math::Matrix3d R_new = worldC * R * bodyC.Inverse();
    gz::math::Quaterniond q_new(R_new);   // does not auto-normalize
    q_new.Normalize();
    return q_new;
}

/**
 * @brief Change the position and attitude of a pose between conventions.
 * @param pose Source pose.
 * @param worldC World-frame axis conversion matrix.
 * @param bodyC Body-frame axis conversion matrix.
 * @return The pose in the target convention.
 */
gz::math::Pose3d poseChangeFrame(
    const gz::math::Pose3d& pose,
    const gz::math::Matrix3d& worldC,
    const gz::math::Matrix3d& bodyC)
{
    return gz::math::Pose3d(
        worldC * pose.Pos(),
        quatChangeFrame(pose.Rot(), worldC, bodyC));
}

// Ordinary body-frame vector (e.g. linear velocity): just the relabeling,
// no extra sign.
/**
 * @brief Relabel an ordinary vector from FLU to the target body frame.
 */
inline gz::math::Vector3d vecBodyChangeFrame(
    const gz::math::Vector3d& v, const gz::math::Matrix3d& bodyC)
{
    return bodyC * v;
}

// Body-frame pseudovector (e.g. angular velocity): picks up an extra sign
// of det(bodyC) relative to an ordinary vector whenever bodyC flips
// handedness. For kBodyFluFrd (det=+1) this is a no-op; for kBodyFluFul and
// kBodyFluFru (det=-1 each) it is not, which is why angular velocity and
// linear velocity need separate helpers even though they look similar.
/**
 * @brief Relabel a body-frame pseudovector, including handedness correction.
 */
inline gz::math::Vector3d pseudoVecBodyChangeFrame(
    const gz::math::Vector3d& v, const gz::math::Matrix3d& bodyC)
{
    return bodyC.Determinant() * (bodyC * v);
}

// this->lin_vel / this->ang_vel (ENU_FLU convention) are stored in the
// ENU ground (WORLD) frame. Every other convention in this file stores
// body-frame velocities. Converting world -> body-of-target therefore
// needs an extra step that pure axis relabeling doesn't: first undo the
// vehicle's own attitude rotation to get the velocity in FLU body-frame
// components, THEN relabel those FLU components into the target's body
// axes.
/**
 * @brief Convert a world-frame velocity to the target body frame.
 * @param v_world_enu Velocity in the ENU world frame.
 * @param q_enu_attitude Vehicle attitude in the ENU_FLU convention.
 * @param bodyC Body-frame axis conversion matrix.
 * @param isPseudoVector Whether the velocity is an angular pseudovector.
 * @return The velocity in the target body frame.
 */
gz::math::Vector3d worldVelToTargetBody(
    const gz::math::Vector3d& v_world_enu,
    const gz::math::Quaterniond& q_enu_attitude,
    const gz::math::Matrix3d& bodyC,
    bool isPseudoVector)
{
    const gz::math::Vector3d v_body_flu =
        q_enu_attitude.RotateVectorReverse(v_world_enu);
    return isPseudoVector ? pseudoVecBodyChangeFrame(v_body_flu, bodyC)
                          : vecBodyChangeFrame(v_body_flu, bodyC);
}

// Inverse of the above: a body-frame velocity in some target convention
// (e.g. xdyn's FRD uvw/pqr) needs relabeling into FLU body-frame
// components, then rotating by the (already-computed) ENU attitude to
// land in ENU_FLU's world-frame velocity storage.
/**
 * @brief Convert a target body-frame velocity to the ENU world frame.
 * @param v_body_target Velocity in the target body frame.
 * @param q_enu_attitude Vehicle attitude in the ENU_FLU convention.
 * @param bodyC Body-frame axis conversion matrix.
 * @param isPseudoVector Whether the velocity is an angular pseudovector.
 * @return The velocity in the ENU world frame.
 */
gz::math::Vector3d targetBodyVelToWorld(
    const gz::math::Vector3d& v_body_target,
    const gz::math::Quaterniond& q_enu_attitude,
    const gz::math::Matrix3d& bodyC,
    bool isPseudoVector)
{
    // bodyC is self-inverse, so the same matrix relabels target->FLU.
    const gz::math::Vector3d v_body_flu =
        isPseudoVector ? pseudoVecBodyChangeFrame(v_body_target, bodyC)
                       : vecBodyChangeFrame(v_body_target, bodyC);
    return q_enu_attitude.RotateVector(v_body_flu);
}

// ---------------------------------------------------------------------------
// NED_FRD-specific quaternion shortcut (kept from the original file).
//
// This is mathematically equivalent to
//   quatChangeFrame(q, kWorldEnuNed, kBodyFluFrd)
// and gives identical results -- it exists as a documented special case
// because BOTH kWorldEnuNed and kBodyFluFrd happen to be proper rotations
// (det = +1 each: NED and ENU are both right-handed, and so are FRD and
// FLU), so each has its own quaternion, and the change-of-frame collapses
// to a plain product of the two, with no matrix round-trip needed. That
// shortcut does NOT exist for EUN_FUL/NEU_FRU below, where individual
// pieces are improper (det = -1) and therefore have no quaternion at all.
// ---------------------------------------------------------------------------
static const gz::math::Quaterniond q_ned_to_enu(0.0, 0.5 * sqrt(2.0), 0.5 * sqrt(2.0), 0.0);
static const gz::math::Quaterniond q_flu_to_frd(0.0, 1.0, 0.0, 0.0);

/** @brief Convert an attitude quaternion from NED_FRD to ENU_FLU. */
gz::math::Quaterniond quatNedToEnu(const gz::math::Quaterniond& q_ned)
{
    return q_ned_to_enu * q_ned * q_flu_to_frd;
}

/** @brief Convert an attitude quaternion from ENU_FLU to NED_FRD. */
gz::math::Quaterniond quatEnuToNed(const gz::math::Quaterniond& q_enu)
{
    return q_ned_to_enu * q_enu * q_flu_to_frd;
}

// ---------------------------------------------------------------------------

/** @brief Convert this GAZEBO state to xdyn's NED_FRD convention. */
VesselInformation VesselInformation::to_xdyn() const
{
    if (convention != Convention::GAZEBO)
        throw std::runtime_error("Invalid convention");

    VesselInformation v;
    v.convention = Convention::NED_FRD;
    v.time = time;
    v.entity = entity;

    v.pose = gz::math::Pose3d(
        kWorldEnuNed * pose.Pos(),
        quatEnuToNed(pose.Rot()));

    // Stored velocities are ENU_FLU world-frame; xdyn wants body(FRD)-frame.
    v.lin_vel = worldVelToTargetBody(lin_vel, pose.Rot(), kBodyFluFrd, /*isPseudoVector=*/false);
    v.ang_vel = worldVelToTargetBody(ang_vel, pose.Rot(), kBodyFluFrd, /*isPseudoVector=*/true);

    return v;
}

/** @brief Convert this GAZEBO state to Unity's EUN_FUL convention. */
VesselInformation VesselInformation::to_unity() const
{
    if (convention != Convention::GAZEBO)
        throw std::runtime_error("Invalid convention");

    VesselInformation v;
    v.convention = Convention::EUN_FUL;
    v.time = time;
    v.entity = entity;

    v.pose = poseChangeFrame(pose, kWorldEnuEun, kBodyFluFul);

    v.lin_vel = worldVelToTargetBody(lin_vel, pose.Rot(), kBodyFluFul, /*isPseudoVector=*/false);
    v.ang_vel = worldVelToTargetBody(ang_vel, pose.Rot(), kBodyFluFul, /*isPseudoVector=*/true);

    return v;
}

/** @brief Convert this GAZEBO state to Unreal's NEU_FRU convention. */
VesselInformation VesselInformation::to_unreal() const
{
    if (convention != Convention::GAZEBO)
        throw std::runtime_error("Invalid convention");

    VesselInformation v;
    v.convention = Convention::NEU_FRU;
    v.time = time;
    v.entity = entity;

    // No quaternion-constant shortcut here (see block comment above
    // kBodyFluFru): worldC and bodyC differ, so this must go through
    // quatChangeFrame's rotation-matrix path.
    v.pose = poseChangeFrame(pose, kWorldEnuNeu, kBodyFluFru);

    v.lin_vel = worldVelToTargetBody(lin_vel, pose.Rot(), kBodyFluFru, /*isPseudoVector=*/false);
    v.ang_vel = worldVelToTargetBody(ang_vel, pose.Rot(), kBodyFluFru, /*isPseudoVector=*/true);

    return v;
}

/**
 * @brief Convert xdyn NED_FRD data to a GAZEBO state.
 */
VesselInformation VesselInformation::from_xdyn(
        const gz::math::Vector3d& ned_xyz,
        const gz::math::Quaterniond& ned_quaternion,
        const gz::math::Vector3d& ned_uvw,   // body(FRD)-frame linear velocity
        const gz::math::Vector3d& ned_pqr)   // body(FRD)-frame angular velocity
{
    VesselInformation s;
    s.convention = Convention::GAZEBO;

    s.pose = gz::math::Pose3d(
        kWorldEnuNed * ned_xyz,     // kWorldEnuNed is self-inverse
        quatNedToEnu(ned_quaternion));

    // xdyn's uvw/pqr are body(FRD)-frame; ENU_FLU stores world-frame
    // velocities, so relabel FRD->FLU, then rotate into the world frame
    // using the attitude we just computed (s.pose.Rot() is now ENU_FLU's).
    s.lin_vel = targetBodyVelToWorld(ned_uvw, s.pose.Rot(), kBodyFluFrd, /*isPseudoVector=*/false);
    s.ang_vel = targetBodyVelToWorld(ned_pqr, s.pose.Rot(), kBodyFluFrd, /*isPseudoVector=*/true);

    return s;
}
