#include "Telemetry.h"

Telemetry::Telemetry(RobotContainer& container) : container(container)
{
    // frc::SmartDashboard::PutData(climbersTableName + "/1", &container.climber1);
    // frc::SmartDashboard::PutData(climbersTableName + "/2", &container.climber2);
    frc::SmartDashboard::PutData(drivetrainTableName, &container.drivetrain);
    frc::SmartDashboard::PutData(drivetrainTableName + "/Align Heading Controller", &container.alignToHub.HeadingController);
    frc::SmartDashboard::PutData(hopperTableName + "/" + floorTableName, &container.floor);
    frc::SmartDashboard::PutData(hopperTableName + "/" + feederTableName, &container.feeder);
    frc::SmartDashboard::PutData(launcherTableName, &container.launcher);
    frc::SmartDashboard::PutData(intakePivotTableName, &container.intakePivot);
    frc::SmartDashboard::PutData(intakePivotTableName + "/Controller", &container.intakePivot.positionController);
    frc::SmartDashboard::PutData(intakeRollerTableName, &container.intakeRoller);
    frc::SmartDashboard::PutData(genericTableName + "/Auto Chooser", &container.autoChooser);
    frc::SmartDashboard::PutData(genericTableName + "/field", &field);
}

void Telemetry::UpdateTelemetry()
{
    swerve::impl::SwerveDrivetrainImpl::SwerveDriveState state = container.drivetrain.GetState();
    drivetrainPose.Set(state.Pose);
    field.SetRobotPose(state.Pose);
    // moduleStates.Set(state.ModuleStates);
    // moduleTargets.Set(state.ModuleTargets);
    // modulePositions.Set(state.ModulePositions);
    // chassisSpeeds.Set(state.Speeds);
    // autoChassisSpeeds.Set(container.autonSetSpeeds);
    
    targetPublisher.Set(container.target == Targets::Hub ? "Hub" : container.target == Targets::Pass ? "Pass" : "Manual");
    manualSetSpeedPublisher.Set(container.launcherSetSpeed.value());
    alignmentWithinTolerance.Set(container.IsAlignmentWithinTolerances());
    robotVoltagePublisher.Set(frc::RobotController::GetBatteryVoltage().value());
    
    // turretCamPose.Set(frc::Pose3d{state.Pose}.TransformBy(VisionConstants::TurretCamera::kRobotToCamera));
    // turretVisionPosePublisher.Set(container.turretVisionPose);
    // turretVisionTargetsPublisher.Set(container.turretVisionTargets);
    // backCamLeftPose.Set(frc::Pose3d{state.Pose}.TransformBy(VisionConstants::BackCameraLeft::kRobotToCamera));
    // backCamLeftVisionPosePublisher.Set(container.backCameraLeftPose);
    // backCamLeftVisionTargetsPublisher.Set(container.backCameraLeftTargets);
    // backCamRightPose.Set(frc::Pose3d{state.Pose}.TransformBy(VisionConstants::BackCameraRight::kRobotToCamera));
    // backCamRightVisionPosePublisher.Set(container.backCameraRightPose);
    // backCamRightVisionTargetsPublisher.Set(container.backCameraRightTargets);

    // intakePose.Set(frc::Pose3d{0.31_m, 0_m, 0.28_m, frc::Rotation3d{0_deg, container.intakePivot.GetAngle(), 0_deg}});

}