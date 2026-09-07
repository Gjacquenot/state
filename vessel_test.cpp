#include "vessel.hpp"
#include <gz/math/Rand.hh>
#include <gtest/gtest.h>
#include <stdexcept>

// Returns a random Vector3d with each component drawn uniformly from [min, max].
gz::math::Vector3d RandomVector3d(double min = -10.0, double max = 10.0)
{
    return gz::math::Vector3d(
        gz::math::Rand::DblUniform(min, max),
        gz::math::Rand::DblUniform(min, max),
        gz::math::Rand::DblUniform(min, max));
}

// Returns a random unit quaternion (uniformly distributed on SO(3)).
// Uses Shoemake's method for a proper uniform random rotation,
// rather than naive random roll/pitch/yaw (which is NOT uniform).
gz::math::Quaterniond RandomQuaternion()
{
    const double u1 = gz::math::Rand::DblUniform(0.0, 1.0);
    const double u2 = gz::math::Rand::DblUniform(0.0, 2.0 * GZ_PI);
    const double u3 = gz::math::Rand::DblUniform(0.0, 2.0 * GZ_PI);

    const double sqrt1MinusU1 = std::sqrt(1.0 - u1);
    const double sqrtU1 = std::sqrt(u1);

    return gz::math::Quaterniond(
        sqrt1MinusU1 * std::sin(u2),   // w
        sqrt1MinusU1 * std::cos(u2),   // x
        sqrtU1 * std::sin(u3),         // y
        sqrtU1 * std::cos(u3));        // z
}



class VesselInformationTest : public ::testing::Test {
protected:
	VesselInformation vessel;

	void SetUp() override
	{
		vessel.time = 12.5;
        vessel.convention = Convention::GAZEBO;
		vessel.pose = gz::math::Pose3d(
			gz::math::Vector3d(10.0, 20.0, 30.0),
			gz::math::Quaterniond::Identity);
		vessel.lin_vel = gz::math::Vector3d(1.0, 2.0, 3.0);
		vessel.ang_vel = gz::math::Vector3d(4.0, 5.0, 6.0);
	}
};

TEST_F(VesselInformationTest, HasExpectedDefaultValues)
{
	VesselInformation default_vessel;

	EXPECT_EQ(default_vessel.convention, Convention::GAZEBO);
	EXPECT_DOUBLE_EQ(default_vessel.time, 0.0);
	EXPECT_EQ(default_vessel.pose, gz::math::Pose3d());
	EXPECT_EQ(default_vessel.lin_vel, gz::math::Vector3d::Zero);
	EXPECT_EQ(default_vessel.ang_vel, gz::math::Vector3d::Zero);
}

TEST_F(VesselInformationTest, ConvertsToXdyn)
{
	const VesselInformation converted = vessel.to_xdyn();

	EXPECT_EQ(converted.convention, Convention::NED_FRD);
	EXPECT_DOUBLE_EQ(converted.time, vessel.time);
	EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(20.0, 10.0, -30.0));
	EXPECT_EQ(converted.lin_vel, gz::math::Vector3d(1.0, -2.0, -3.0));
	EXPECT_EQ(converted.ang_vel, gz::math::Vector3d(4.0, -5.0, -6.0));
}

TEST_F(VesselInformationTest, ConvertsToUnity)
{
	const VesselInformation converted = vessel.to_unity();

	EXPECT_EQ(converted.convention, Convention::EUN_FUL);
	EXPECT_DOUBLE_EQ(converted.time, vessel.time);
	EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(10.0, 30.0, 20.0));
	EXPECT_EQ(converted.lin_vel, gz::math::Vector3d(1.0, 3.0, 2.0));
	EXPECT_EQ(converted.ang_vel, gz::math::Vector3d(-4.0, -6.0, -5.0));
}

TEST_F(VesselInformationTest, ConvertsToUnreal)
{
	const VesselInformation converted = vessel.to_unreal();

	EXPECT_EQ(converted.convention, Convention::NEU_FRU);
	EXPECT_DOUBLE_EQ(converted.time, vessel.time);
	EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(20.0, 10.0, 30.0));
	EXPECT_EQ(converted.lin_vel, gz::math::Vector3d(1.0, -2.0, 3.0));
	EXPECT_EQ(converted.ang_vel, gz::math::Vector3d(-4.0, 5.0, -6.0));
}

TEST_F(VesselInformationTest, ConvertsFromXdyn)
{
	const VesselInformation converted = VesselInformation::from_xdyn(
		gz::math::Vector3d(20.0, 10.0, -30.0),
		gz::math::Quaterniond::Identity,
		gz::math::Vector3d(1.0, -2.0, -3.0),
		gz::math::Vector3d(4.0, -5.0, -6.0));

    const gz::math::Quaterniond expected_quaternion = gz::math::Quaterniond(0,1.0/sqrt(2.0),1.0/sqrt(2.0),0) * gz::math::Quaterniond(0,1,0,0);
	EXPECT_EQ(converted.convention, Convention::GAZEBO);
	EXPECT_EQ(converted.pose.Pos(), gz::math::Vector3d(10.0, 20.0, 30.0));
	EXPECT_EQ(converted.pose.Rot(), expected_quaternion);
	EXPECT_EQ(converted.lin_vel, expected_quaternion.RotateVector(gz::math::Vector3d(1.0, 2.0, 3.0)));
	EXPECT_EQ(converted.ang_vel, expected_quaternion.RotateVector(gz::math::Vector3d(4.0, 5.0, 6.0)));
}

TEST_F(VesselInformationTest, ConvertsFromXdynRandom)
{
	const gz::math::Vector3d ned_xyz = RandomVector3d();
	const gz::math::Quaterniond ned_quat = RandomQuaternion();
	const gz::math::Vector3d ned_uvw = RandomVector3d();
	const gz::math::Vector3d ned_pqr = RandomVector3d();
	const VesselInformation converted = VesselInformation::from_xdyn(
		ned_xyz,
		ned_quat,
		ned_uvw,
		ned_pqr);

    const gz::math::Quaterniond expected_quaternion = gz::math::Quaterniond(0,1.0/sqrt(2.0),1.0/sqrt(2.0),0) * ned_quat *gz::math::Quaterniond(0,1,0,0);
	EXPECT_EQ(converted.convention, Convention::GAZEBO);
    EXPECT_EQ(converted.pose.Pos(),
            gz::math::Vector3d(ned_xyz.Y(), ned_xyz.X(), -ned_xyz.Z()));
	EXPECT_EQ(converted.pose.Rot(), expected_quaternion);
    EXPECT_EQ(converted.lin_vel,
            expected_quaternion.RotateVector(
                gz::math::Vector3d(ned_uvw.X(), -ned_uvw.Y(), -ned_uvw.Z())));
    EXPECT_EQ(converted.ang_vel,
            expected_quaternion.RotateVector(
            gz::math::Vector3d(ned_pqr.X(), -ned_pqr.Y(), -ned_pqr.Z())));
}

TEST_F(VesselInformationTest, ConvertsToXdynRandom)
{
	const gz::math::Vector3d xyz = RandomVector3d();
	const gz::math::Quaterniond quat = RandomQuaternion();
	const gz::math::Vector3d uvw = RandomVector3d();
	const gz::math::Vector3d pqr = RandomVector3d();
	const VesselInformation state = VesselInformation(Convention::GAZEBO, 0.0, gz::math::Pose3d(xyz, quat), uvw, pqr);
	const VesselInformation converted = state.to_xdyn();
    const gz::math::Quaterniond expected_quaternion = gz::math::Quaterniond(0,1.0/sqrt(2.0),1.0/sqrt(2.0),0) * quat *gz::math::Quaterniond(0,1,0,0);
	EXPECT_EQ(converted.convention, Convention::NED_FRD);
    EXPECT_EQ(converted.pose.Pos(),
            gz::math::Vector3d(xyz.Y(), xyz.X(), -xyz.Z()));
	EXPECT_EQ(converted.pose.Rot(), expected_quaternion);
    const auto body_uvw = quat.RotateVectorReverse(uvw);
    EXPECT_EQ(converted.lin_vel,
        gz::math::Vector3d(body_uvw.X(), -body_uvw.Y(), -body_uvw.Z()));
    const auto body_pqr = quat.RotateVectorReverse(pqr);
    EXPECT_EQ(converted.ang_vel,
            gz::math::Vector3d(body_pqr.X(), -body_pqr.Y(), -body_pqr.Z()));
}

TEST_F(VesselInformationTest, RejectsConversionsFromOtherConventions)
{
	vessel.convention = Convention::NED_FRD;

	EXPECT_THROW(vessel.to_xdyn(), std::runtime_error);
	EXPECT_THROW(vessel.to_unity(), std::runtime_error);
	EXPECT_THROW(vessel.to_unreal(), std::runtime_error);
}
