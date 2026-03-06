#pragma once
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
#include <frc/geometry/Rotation2d.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
using namespace ctre::phoenix6::swerve::requests;
using namespace ctre::phoenix6::swerve;
using namespace frc;
namespace ctre::phoenix6::swerve::requests
{
    /**
     * \brief Drives the swerve drivetrain in a field-centric manner, maintaining a
     * specified heading angle to ensure the robot is facing the desired direction
     *
     * When users use this request, they specify the direction the robot should
     * travel oriented against the field, and the direction the robot should be facing.
     *
     * An example scenario is that the robot is oriented to the east, the VelocityX
     * is +5 m/s, VelocityY is 0 m/s, and TargetDirection is 180 degrees.
     * In this scenario, the robot would drive northward at 5 m/s and turn clockwise
     * to a target of 180 degrees.
     *
     * This control request is especially useful for autonomous control, where the
     * robot should be facing a changing direction throughout the motion.
     */
    class FieldCentricFacingAngleProfiled : public SwerveRequest {
    public:
        /**
         * \brief The velocity in the X direction. X is defined as forward according
         * to WPILib convention, so this determines how fast to travel forward.
         */
        units::meters_per_second_t VelocityX = 0_mps;
        /**
         * \brief The velocity in the Y direction. Y is defined as to the left
         * according to WPILib convention, so this determines how fast to travel to
         * the left.
         */
        units::meters_per_second_t VelocityY = 0_mps;
        /**
         * \brief The desired direction to face.
         * 0 Degrees is defined as in the direction of the X axis.
         * As a result, a TargetDirection of 90 degrees will point along
         * the Y axis, or to the left.
         */
        Rotation2d TargetDirection{};
        /**
         * \brief The rotational rate feedforward to add to the output of the heading
         * controller, in degrees per second. When using a motion profile for the
         * target direction, this can be set to the current velocity reference of
         * the profile.
         */
        units::degrees_per_second_t TargetRateFeedforward = 0_deg_per_s;

        /**
         * \brief The allowable deadband of the request.
         */
        units::meters_per_second_t Deadband = 0_mps;
        /**
         * \brief The rotational deadband of the request.
         */
        units::degrees_per_second_t RotationalDeadband = 0_deg_per_s;
        /**
         * \brief The maximum absolute rotational rate to allow.
         * Setting this to 0 results in no cap to rotational rate.
         */
        units::degrees_per_second_t MaxAbsRotationalRate = 0_deg_per_s;
        /**
         * \brief The center of rotation the robot should rotate around. This is
         * (0,0) by default, which will rotate around the center of the robot.
         */
        Translation2d CenterOfRotation{};

        /**
         * \brief The type of control request to use for the drive motor.
         */
        impl::DriveRequestType DriveRequestType = impl::DriveRequestType::OpenLoopVoltage;
        /**
         * \brief The type of control request to use for the steer motor.
         */
        impl::SteerRequestType SteerRequestType = impl::SteerRequestType::Position;
        /**
         * \brief Whether to desaturate wheel speeds before applying.
         * For more information, see the documentation of impl#SwerveDriveKinematics#DesaturateWheelSpeeds.
         */
        bool DesaturateWheelSpeeds = true;

        /**
         * \brief The perspective to use when determining which direction is forward.
         */
        ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue::OperatorPerspective;

        /**
         * \brief The PID controller used to maintain the desired heading.
         * Users can specify the PID gains to change how aggressively to maintain
         * heading.
         *
         * This PID controller operates on heading degrees and outputs a target
         * rotational rate in degrees per second. Note that continuous input should
         * be enabled on the range [-180, 180].
         */
        ProfiledPIDController<units::degree> HeadingController{0, 0, 0, TrapezoidProfile<units::degree>::Constraints{0_deg_per_s, 0_deg_per_s_sq}};

        units::degree_t Tolerance;


        FieldCentricFacingAngleProfiled()
        {
            HeadingController.EnableContinuousInput(-180_deg, 180_deg);
        }

        ctre::phoenix::StatusCode Apply(SwerveRequest::ControlParameters const &parameters, std::span<std::unique_ptr<impl::SwerveModuleImpl> const> modulesToApply) override;

        FieldCentricFacingAngleProfiled &WithTolerance(units::degree_t tolerance)
        {
            this->HeadingController.SetTolerance(tolerance);
            this->Tolerance = std::move(tolerance);
            return *this;
        }

        /**
         * \brief Modifies the PID gains of the HeadingController parameter and returns itself.
         *
         * Sets the proportional, integral, and differential coefficients used to maintain
         * the desired heading. Users can specify the PID gains to change how aggressively to
         * maintain heading.
         *
         * This PID controller operates on heading degrees and outputs a target
         * rotational rate in degrees per second.
         *
         * \param kP The proportional coefficient; must be >= 0
         * \param kI The integral coefficient; must be >= 0
         * \param kD The differential coefficient; must be >= 0
         * \returns this object
         */
        FieldCentricFacingAngleProfiled &WithHeadingPID(double kP, double kI, double kD)
        {
            this->HeadingController.SetPID(kP, kI, kD);
            return *this;
        }

        FieldCentricFacingAngleProfiled &WithConstraints(units::degrees_per_second_t velocity, units::angular_acceleration::degrees_per_second_squared_t acceleration)
        {
            this->HeadingController.SetConstraints(TrapezoidProfile<units::degree>::Constraints{velocity, acceleration});
            return *this;
        }

        /**
         * \brief Modifies the VelocityX parameter and returns itself.
         *
         * The velocity in the X direction. X is defined as forward according to
         * WPILib convention, so this determines how fast to travel forward.
         *
         * \param newVelocityX Parameter to modify
         * \returns this object
         */
        FieldCentricFacingAngleProfiled &WithVelocityX(units::meters_per_second_t newVelocityX)
        {
            this->VelocityX = std::move(newVelocityX);
            return *this;
        }

        /**
         * \brief Modifies the VelocityY parameter and returns itself.
         *
         * The velocity in the Y direction. Y is defined as to the left according
         * to WPILib convention, so this determines how fast to travel to the
         * left.
         *
         * \param newVelocityY Parameter to modify
         * \returns this object
         */
        FieldCentricFacingAngleProfiled &WithVelocityY(units::meters_per_second_t newVelocityY)
        {
            this->VelocityY = std::move(newVelocityY);
            return *this;
        }

        /**
         * \brief Modifies the VelocityY parameter and returns itself.
         *
         * The desired direction to face. 0 Degrees is defined as in the direction of
         * the X axis. As a result, a TargetDirection of 90 degrees will point along
         * the Y axis, or to the left.
         *
         * \param newTargetDirection Parameter to modify
         * \returns this object
         */
        FieldCentricFacingAngleProfiled &WithTargetDirection(Rotation2d newTargetDirection)
        {
            this->TargetDirection = std::move(newTargetDirection);
            return *this;
        }

        /**
         * \brief Modifies the VelocityY parameter and returns itself.
         *
         * The rotational rate feedforward to add to the output of the heading
         * controller, in degrees per second. When using a motion profile for the
         * target direction, this can be set to the current velocity reference of
         * the profile.
         *
         * \param newTargetRateFeedforward Parameter to modify
         * \returns this object
         */
        FieldCentricFacingAngleProfiled &WithTargetRateFeedforward(units::degrees_per_second_t newTargetRateFeedforward)
        {
            this->TargetRateFeedforward = std::move(newTargetRateFeedforward);
            return *this;
        }

        /**
         * \brief Modifies the Deadband parameter and returns itself.
         *
         * The allowable deadband of the request.
         *
         * \param newDeadband Parameter to modify
         * \returns this object
         */
        FieldCentricFacingAngleProfiled &WithDeadband(units::meters_per_second_t newDeadband)
        {
            this->Deadband = std::move(newDeadband);
            return *this;
        }

        /**
         * \brief Modifies the RotationalDeadband parameter and returns itself.
         *
         * The rotational deadband of the request.
         *
         * \param newRotationalDeadband Parameter to modify
         * \returns this object
         */
        FieldCentricFacingAngleProfiled &WithRotationalDeadband(units::degrees_per_second_t newRotationalDeadband)
        {
            this->RotationalDeadband = std::move(newRotationalDeadband);
            return *this;
        }

        /**
         * \brief Modifies the MaxAbsRotationalRate parameter and returns itself.
         *
         * The maximum absolute rotational rate to allow.
         * Setting this to 0 results in no cap to rotational rate.
         *
         * \param newMaxAbsRotationalRate Parameter to modify
         * \returns this object
         */
        FieldCentricFacingAngleProfiled &WithMaxAbsRotationalRate(units::degrees_per_second_t newMaxAbsRotationalRate)
        {
            this->MaxAbsRotationalRate = std::move(newMaxAbsRotationalRate);
            return *this;
        }

        /**
         * \brief Modifies the CenterOfRotation parameter and returns itself.
         *
         * The center of rotation the robot should rotate around. This is (0,0) by
         * default, which will rotate around the center of the robot.
         *
         * \param newCenterOfRotation Parameter to modify
         * \returns this object
         */
        FieldCentricFacingAngleProfiled &WithCenterOfRotation(Translation2d newCenterOfRotation)
        {
            this->CenterOfRotation = std::move(newCenterOfRotation);
            return *this;
        }

        /**
         * \brief Modifies the DriveRequestType parameter and returns itself.
         *
         * The type of control request to use for the drive motor.
         *
         * \param newDriveRequestType Parameter to modify
         * \returns this object
         */
        FieldCentricFacingAngleProfiled &WithDriveRequestType(impl::DriveRequestType newDriveRequestType)
        {
            this->DriveRequestType = std::move(newDriveRequestType);
            return *this;
        }

        /**
         * \brief Modifies the SteerRequestType parameter and returns itself.
         *
         * The type of control request to use for the steer motor.
         *
         * \param newSteerRequestType Parameter to modify
         * \returns this object
         */
        FieldCentricFacingAngleProfiled &WithSteerRequestType(impl::SteerRequestType newSteerRequestType)
        {
            this->SteerRequestType = std::move(newSteerRequestType);
            return *this;
        }

        /**
         * \brief Modifies the DesaturateWheelSpeeds parameter and returns itself.
         *
         * Whether to desaturate wheel speeds before applying. For more information, see
         * the documentation of impl#SwerveDriveKinematics#DesaturateWheelSpeeds.
         *
         * \param newDesaturateWheelSpeeds Parameter to modify
         * \returns this object
         */
        FieldCentricFacingAngleProfiled &WithDesaturateWheelSpeeds(bool newDesaturateWheelSpeeds)
        {
            this->DesaturateWheelSpeeds = std::move(newDesaturateWheelSpeeds);
            return *this;
        }

        /**
         * \brief Modifies the ForwardPerspective parameter and returns itself.
         *
         * The perspective to use when determining which direction is forward.
         *
         * \param newForwardPerspective Parameter to modify
         * \returns this object
         */
        FieldCentricFacingAngleProfiled &WithForwardPerspective(ForwardPerspectiveValue newForwardPerspective)
        {
            this->ForwardPerspective = std::move(newForwardPerspective);
            return *this;
        }
    };
}