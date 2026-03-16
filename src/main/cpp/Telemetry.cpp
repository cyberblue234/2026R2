#include "Telemetry.h"

Telemetry::Telemetry(RobotContainer& container) : container(container)
{
    frc::SmartDashboard::PutData(climbersTableName + "/1", &container.climber1);
    frc::SmartDashboard::PutData(climbersTableName + "/2", &container.climber2);
    frc::SmartDashboard::PutData(drivetrainTableName, &container.drivetrain);
    frc::SmartDashboard::PutData(drivetrainTableName + "/Align Heading Controller", &container.alignToHub.HeadingController);
    frc::SmartDashboard::PutData(hopperTableName + "/" + floorTableName, &container.floor);
    frc::SmartDashboard::PutData(hopperTableName + "/" + feederTableName, &container.feeder);
    frc::SmartDashboard::PutData(launcherTableName, &container.launcher);
    frc::SmartDashboard::PutData(intakePivotTableName, &container.intakePivot);
    frc::SmartDashboard::PutData(intakePivotTableName + "/Controller", &container.intakePivot.positionController);
    frc::SmartDashboard::PutData(intakeRollerTableName, &container.intakeRoller);
    
}

void Telemetry::UpdateTelemetry()
{
    swerve::impl::SwerveDrivetrainImpl::SwerveDriveState state = container.drivetrain.GetState();
    drivetrainPose.Set(state.Pose);
    moduleStates.Set(state.ModuleStates);
    moduleTargets.Set(state.ModuleTargets);
    modulePositions.Set(state.ModulePositions);
    chassisSpeeds.Set(state.Speeds);
    
    targetPublisher.Set(container.target == Targets::Hub ? "Hub" : container.target == Targets::Pass ? "Pass" : "Manual");
    frc::SmartDashboard::PutNumber(genericTableName + "/Manual Set Speed", container.launcherSetSpeed.value());
    frc::SmartDashboard::PutBoolean(genericTableName + "/Is Alignment Within Tolerances", container.IsAlignmentWithinTolerances());
    
    turretCamPose.Set(frc::Pose3d{state.Pose}.TransformBy(VisionConstants::TurretCamera::kRobotToCamera));
    turretVisionPosePublisher.Set(container.turretVisionPose);
    turretVisionTargetsPublisher.Set(container.turretVisionTargets);
    backCam1Pose.Set(frc::Pose3d{state.Pose}.TransformBy(VisionConstants::BackCamera1::kRobotToCamera));
    backCam1VisionPosePublisher.Set(container.backCamera1Pose);
    backCam1VisionTargetsPublisher.Set(container.backCamera1Targets);

    intakePose.Set(frc::Pose3d{0.31_m, 0_m, 0.28_m, frc::Rotation3d{0_deg, container.intakePivot.GetAngle(), 0_deg}});

}