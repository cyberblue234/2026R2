#include "generated/FieldCentricFacingAngleProfiled.h"

ctre::phoenix::StatusCode FieldCentricFacingAngleProfiled::Apply(SwerveRequest::ControlParameters const &parameters, std::span<std::unique_ptr<impl::SwerveModuleImpl> const> modulesToApply)
{
    TargetRateFeedforward = HeadingController.GetSetpoint().velocity;
    units::degrees_per_second_t toApplyOmega = TargetRateFeedforward +
        units::degrees_per_second_t{HeadingController.Calculate(parameters.currentPose.Rotation().Degrees(), TargetDirection.Degrees())};
    frc::SmartDashboard::PutNumber("targetSetpoint", HeadingController.GetSetpoint().position.value());
    frc::SmartDashboard::PutNumber("targetGoal", HeadingController.GetGoal().position.value());
    if (MaxAbsRotationalRate > 0_deg_per_s) {
        if (toApplyOmega > MaxAbsRotationalRate) {
            toApplyOmega = MaxAbsRotationalRate;
        } else if (toApplyOmega < -MaxAbsRotationalRate) {
            toApplyOmega = -MaxAbsRotationalRate;
        }
    }
    
    return FieldCentric{}
        .WithVelocityX(VelocityX)
        .WithVelocityY(VelocityY)
        .WithRotationalRate(toApplyOmega)
        .WithDeadband(Deadband)
        .WithRotationalDeadband(RotationalDeadband)
        .WithCenterOfRotation(CenterOfRotation)
        .WithDriveRequestType(DriveRequestType)
        .WithSteerRequestType(SteerRequestType)
        .WithDesaturateWheelSpeeds(DesaturateWheelSpeeds)
        .WithForwardPerspective(ForwardPerspective)
        .Apply(parameters, modulesToApply);
}