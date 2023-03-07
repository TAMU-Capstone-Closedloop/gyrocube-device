#include "informants/transformers/coordinate_frame.hpp"

#include "utils/math/transform_setup.hpp"
#include "utils/robot_specific_inc.hpp"

using namespace src::Utils::MatrixHelper;

namespace src::Informants {

// Constructor
CoordinateFrame::CoordinateFrame(Vector3f origin, Matrix3f orientation) : origin(origin), orientation(orientation) {
    updateTransform();
}

// Deconstructor I Guess
CoordinateFrame::~CoordinateFrame() {}

// Update Transforms in/out
void CoordinateFrame::updateTransform() {
    this->transformOut = initTransform(this->orientation, this->origin);
    this->transformIn = invertTransform(this->transformOut);
}
void CoordinateFrame::setOrientation(Matrix3f R) {
    this->orientation = R;
    updateTransform();
}
void CoordinateFrame::rotateFrame(Matrix3f R) {
    this->orientation = this->orientation * R;
    updateTransform();
}
void CoordinateFrame::moveOrigin(Vector3f r) {
    this->origin = this->origin + r;
    updateTransform();
}
void CoordinateFrame::setOrigin(Vector3f r) {
    this->origin = r;
    updateTransform();
}

// Getters
Vector3f CoordinateFrame::getOrigin() { return this->origin; }
Matrix3f CoordinateFrame::getOrientation() { return this->orientation; }
Matrix4f CoordinateFrame::getTransformIn() { return this->transformIn; }
Matrix4f CoordinateFrame::getTransformOut() { return this->transformOut; }

// Returns a point in this frame
Vector3f CoordinateFrame::getPoint(int n) { this->points.at(n); }

// Add a point to a frame (to track) -> Point should be rigid w.r.t this frame
void CoordinateFrame::addPoint(Vector3f p) { this->points.push_back(p); }

// Returns a point in the given frame that is stored in the current frame
Vector3f CoordinateFrame::getPointInFrame(CoordinateFrame& f, int n) {
    return homogenousCoordinateCrop(f.transformIn * this->transformOut * homogenousCoordinateExtend(getPoint(n)).asMatrix());
}

}  // namespace src::Informants