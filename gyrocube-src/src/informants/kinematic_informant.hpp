#pragma once

#include <tap/algorithms/contiguous_float.hpp>

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "transformers/robot_frames.hpp"
#include "utils/common_types.hpp"
#include "utils/kinematic_state_vector.hpp"

namespace src {
class Drivers;
}  // namespace src

using namespace src::Utils;

namespace src::Informants {

enum AngularAxis { PITCH_AXIS = 0, ROLL_AXIS = 1, YAW_AXIS = 2 };

class KinematicInformant {
public:
    KinematicInformant(src::Drivers* drivers);
    ~KinematicInformant() = default;

    src::Informants::Transformers::RobotFrames& getRobotFrames() { return robotFrames; }

    void registerSubsystems() {}

    tap::communication::sensors::imu::ImuInterface::ImuState getIMUState();

    void initialize(float imuFrequency, float imukP, float imukI);

    void recalibrateIMU(Vector3f imuCalibrationEuler = {0.0f, 0.0f, 0.0f});

    // Gets raw IMU values, in our coordinate system XYZ (Pitch, Roll, Yaw)
    Vector3f getLocalIMUAngles();
    float getLocalIMUAngle(AngularAxis axis);

    Vector3f getIMUAngularVelocities();
    float getIMUAngularVelocity(AngularAxis axis);

    Vector3f getIMULinearAccelerations();
    float getIMULinearAcceleration(LinearAxis axis);

    Vector3f removeFalseAcceleration(
        Vector<KinematicStateVector, 3> imuLKSV,
        Vector<KinematicStateVector, 3> imuAKSV,
        Vector3f r);

    void updateIMUKinematicStateVector();

    // Returns angle in rad or deg
    float getChassisIMUAngle(AngularAxis axis, AngleUnit unit);

    // Returns angular velocity in rad/s or deg/s
    float getChassisIMUAngularVelocity(AngularAxis axis, AngleUnit unit);

    Vector3f getIMUAngularAccelerations();

    float getIMUAngularAcceleration(AngularAxis axis, AngleUnit unit);
    // Returns nothing!!

    void updateRobotFrames();

private:
    src::Drivers* drivers;

    src::Informants::Transformers::RobotFrames robotFrames;

    KinematicStateVector imuLinearXState;
    KinematicStateVector imuLinearYState;
    KinematicStateVector imuLinearZState;

    KinematicStateVector imuAngularXState;
    KinematicStateVector imuAngularYState;
    KinematicStateVector imuAngularZState;

    modm::Vector<KinematicStateVector, 3> imuLinearState = {imuLinearXState, imuLinearYState, imuLinearZState};
    modm::Vector<KinematicStateVector, 3> imuAngularState = {imuAngularXState, imuAngularYState, imuAngularZState};
};

}  // namespace src::Informants
