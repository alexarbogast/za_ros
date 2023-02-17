#include <za_gazebo/joint.h>
#include <urdf/model.h>
#include <gazebo/physics/Link.hh>

namespace za_gazebo 
{
void Joint::update(const ros::Duration& dt) 
{
    if (not this->handle) { 
        return;
    }

    this->velocity = this->handle->GetVelocity(0);
    double position = this->handle->Position(0);

    ignition::math::Vector3d f;
    switch (this->type)  {
        case urdf::Joint::PRISMATIC:
            this->position = position;
            f = this->handle->GetForceTorque(0).body2Force;
            break;
        case urdf::Joint::REVOLUTE:
        case urdf::Joint::CONTINUOUS:
            this->position += angles::shortest_angular_distance(this->position, position);
            f = this->handle->GetForceTorque(0).body2Torque;
            break;
        default:
            throw std::logic_error("Unknown joint type: " + std::to_string(this->type));
    }
    this->effort = Eigen::Vector3d(f.X(), f.Y(), f.Z()).dot(this->axis);

    if (std::isnan(this->lastVelocity)) {
        this->lastVelocity = this->velocity;
    }
    this->acceleration = (this->velocity - this->lastVelocity) / dt.toSec();
    this->lastVelocity = this->velocity;

    if (std::isnan(this->lastAcceleration)) {
        this->lastAcceleration = this->acceleration;
    }
    this->jerk = (this->acceleration - this->lastAcceleration) / dt.toSec();
    this->lastAcceleration = this->acceleration;
}

double Joint::getDesiredPosition() const 
{
    if (this->control_method == ControlMethod::POSITION or
        this->control_method != ControlMethod::EFFORT) 
    {
        return this->desired_position;
    }
    return this->position;
}

double Joint::getDesiredVelocity() const 
{
    if (this->control_method != ControlMethod::VELOCITY)
    {
        return this->velocity;
    }
    return this->desired_velocity;
}

double Joint::getDesiredAcceleration() const 
{
    if (this->control_method == ControlMethod::EFFORT) {
        return 0;
    }
    return this->acceleration;
}

double Joint::getDesiredTorque() const 
{
    if (this->control_method != ControlMethod::EFFORT) 
    {
        return 0;
    }
    return command;
}

double Joint::getLinkMass() const 
{
    if (not this->handle) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return this->handle->GetChild()->GetInertial()->Mass();
}

bool Joint::isInCollision() const 
{
    return std::abs(this->effort - this->command) > this->collision_threshold;
}

bool Joint::isInContact() const 
{
    return std::abs(this->effort - this->command) > this->contact_threshold;
}
}  // namespace za_gazebo