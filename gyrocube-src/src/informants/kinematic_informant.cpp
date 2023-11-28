#include "kinematic_informant.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::Informants {

KinematicInformant::KinematicInformant(src::Drivers* drivers) : drivers(drivers) {}

void KinematicInformant::initialize(float imuFrequency, float imukP, float imukI) {
    drivers->bmi088.initialize(imuFrequency, imukP, imukI);
}

void KinematicInformant::recalibrateIMU(Vector3f imuCalibrationEuler) {
    // drivers->bmi088.requestRecalibration(imuCalibrationEuler);
    UNUSED(imuCalibrationEuler);
    drivers->bmi088.requestRecalibration();
};

tap::communication::sensors::imu::ImuInterface::ImuState KinematicInformant::getIMUState() {
    return drivers->bmi088.getImuState();
}

Vector3f KinematicInformant::getLocalIMUAngles() {
    Vector3f imuAngles = {
        -drivers->bmi088.getPitch(),  // inverts pitch
        drivers->bmi088.getRoll(),
        drivers->bmi088.getYaw() - 180.0f};  // for some reason yaw is 180.0 degrees rotated
    return imuAngles * (M_PI / 180.0f);      // Convert to rad
}
float KinematicInformant::getLocalIMUAngle(AngularAxis axis) {  // Gets IMU angles in IMU Frame
    switch (axis) {
        case PITCH_AXIS:
            return -modm::toRadian(drivers->bmi088.getPitch());
        case ROLL_AXIS:
            return modm::toRadian(drivers->bmi088.getRoll());
        case YAW_AXIS:
            return modm::toRadian(drivers->bmi088.getYaw() - 180.0f);  // for some reason yaw is 180.0 degrees rotated
    }
    return 0;
}

Vector3f KinematicInformant::getIMUAngularVelocities() {  // Gets IMU Angular Velocity in IMU FRAME
    Vector3f imuAngularVelocities = {-drivers->bmi088.getGy(), drivers->bmi088.getGx(), drivers->bmi088.getGz()};
    return imuAngularVelocities * (M_PI / 180.0f);  // Convert to rad/s
}

float KinematicInformant::getIMUAngularVelocity(AngularAxis axis) {  // Gets IMU angles in IMU Frame
    switch (axis) {
        case PITCH_AXIS:
            return -modm::toRadian(drivers->bmi088.getGy());
        case ROLL_AXIS:
            return modm::toRadian(drivers->bmi088.getGx());
        case YAW_AXIS:
            return modm::toRadian(drivers->bmi088.getGz());
    }
    return 0;
}

// Update IMU Kinematic State Vectors
// All relative to IMU Frame
void KinematicInformant::updateIMUKinematicStateVector() {
    imuLinearState[X_AXIS].updateFromAcceleration(-drivers->bmi088.getAy());
    imuLinearState[Y_AXIS].updateFromAcceleration(drivers->bmi088.getAx());
    imuLinearState[Z_AXIS].updateFromAcceleration(drivers->bmi088.getAz());

    imuAngularState[X_AXIS].updateFromPosition(getLocalIMUAngle(PITCH_AXIS));
    imuAngularState[Y_AXIS].updateFromPosition(getLocalIMUAngle(ROLL_AXIS));
    imuAngularState[Z_AXIS].updateFromPosition(getLocalIMUAngle(YAW_AXIS));

    imuAngularState[X_AXIS].updateFromVelocity(getIMUAngularVelocity(PITCH_AXIS), false);
    imuAngularState[Y_AXIS].updateFromVelocity(getIMUAngularVelocity(ROLL_AXIS), false);
    imuAngularState[Z_AXIS].updateFromVelocity(getIMUAngularVelocity(YAW_AXIS), false);
}

Vector3f KinematicInformant::getIMUAngularAccelerations() {
    float alphax = imuAngularState[X_AXIS].getAcceleration();
    float alphay = imuAngularState[Y_AXIS].getAcceleration();
    float alphaz = imuAngularState[Z_AXIS].getAcceleration();

    Vector3f alpha = {alphax, alphay, alphaz};
    return alpha;
}

Vector3f KinematicInformant::getIMULinearAccelerations() {
    float ax = -drivers->bmi088.getAy();
    float ay = drivers->bmi088.getAx();
    float az = drivers->bmi088.getAz();

    Vector3f a = {ax, ay, az};
    return a;
}

float KinematicInformant::getIMULinearAcceleration(LinearAxis axis) {  // Gets IMU accel in IMU Frame
    switch (axis) {
        case X_AXIS:
            return drivers->bmi088.getAx();
        case Y_AXIS:
            return drivers->bmi088.getAy();
        case Z_AXIS:
            return drivers->bmi088.getAz();
    }
    return 0;
}

float KinematicInformant::getIMUAngularAcceleration(AngularAxis axis, AngleUnit unit) {
    float angularAcceleration = imuAngularState[axis].getAcceleration();

    return unit == AngleUnit::Radians ? angularAcceleration : modm::toDegree(angularAcceleration);
}

Vector3f wDisplay = {0.0f, 0.0f, 0.0f};
Vector3f alphaDisplay = {0.0f, 0.0f, 0.0f};
Vector3f rDisplay = {0.0f, 0.0f, 0.0f};
Vector3f aDisplay = {0.0f, 0.0f, 0.0f};

float aXDisplay = 0.0f;
float aYDisplay = 0.0f;
float aZDisplay = 0.0f;
Vector3f KinematicInformant::removeFalseAcceleration(
    Vector<KinematicStateVector, 3> imuLinearKSV,
    Vector<KinematicStateVector, 3> imuAngularKSV,
    Vector3f r) {
    Vector3f w = {
        imuAngularKSV[X_AXIS].getVelocity(),
        imuAngularKSV[Y_AXIS].getVelocity(),
        imuAngularKSV[Z_AXIS].getVelocity()};

    // Vector3f alpha = {
    //     imuAngularKSV[X_AXIS].getAcceleration(),
    //     imuAngularKSV[Y_AXIS].getAcceleration(),
    //     imuAngularKSV[Z_AXIS].getAcceleration()};
    // was broken so i made it 0, requires investigation on why it's broken (was setting stuff to infinity)

    Vector3f alpha = {0.0f, 0.0f, 0.0f};

    Vector3f a = {
        imuLinearKSV[X_AXIS].getAcceleration(),
        imuLinearKSV[Y_AXIS].getAcceleration(),
        imuLinearKSV[Z_AXIS].getAcceleration()};

    aXDisplay = a.getX();
    aYDisplay = a.getY();
    aZDisplay = a.getZ();

    wDisplay = w;
    alphaDisplay = alpha;
    rDisplay = r;
    aDisplay = a;

    Vector3f linearIMUAcceleration = a - (alpha ^ r) - (w ^ (w ^ r));
    return linearIMUAcceleration;
}

Vector3f linearIMUAccelerationDisplay;
float linearIMUAccelerationXDisplay = 0.0f;
float linearIMUAccelerationYDisplay = 0.0f;
float linearIMUAccelerationZDisplay = 0.0f;

modm::Location2D<float> robotLocationDisplay;
float robotLocationXDisplay = 0.0f;
float robotLocationYDisplay = 0.0f;

void KinematicInformant::updateRobotFrames() {
    // Update IMU Stuff
    updateIMUKinematicStateVector();
}

}  // namespace src::Informants