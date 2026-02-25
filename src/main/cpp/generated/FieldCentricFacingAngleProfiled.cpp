#include "generated/FieldCentricFacingAngleProfiled.h"

ctre::phoenix::StatusCode FieldCentricFacingAngleProfiled::Apply(SwerveRequest::ControlParameters const &parameters, std::span<std::unique_ptr<impl::SwerveModuleImpl> const> modulesToApply)
{
    Rotation2d angleToFace = TargetDirection;
    if (ForwardPerspective == ForwardPerspectiveValue::OperatorPerspective) {
        /* If we're operator perspective, rotate the direction we want to face by the angle */
        angleToFace = angleToFace.RotateBy(parameters.operatorForwardDirection);
    }

    HeadingController.SetGoal(angleToFace.Degrees());
    
    units::degrees_per_second_t toApplyOmega = TargetRateFeedforward +
        units::degrees_per_second_t{HeadingController.Calculate(parameters.currentPose.Rotation().Degrees())};
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