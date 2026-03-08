#include "Telemetry.h"

Telemetry::Telemetry(RobotContainer& container) : container(container)
{
    frc::SmartDashboard::PutData(std::string(nt::NetworkTable::BasenameKey(climbersTable->GetPath())) + "/1", &container.climber1);
    frc::SmartDashboard::PutData(std::string(nt::NetworkTable::BasenameKey(climbersTable->GetPath())) + "/2", &container.climber2);
    frc::SmartDashboard::PutData(nt::NetworkTable::BasenameKey(drivetrainTable->GetPath()), &container.drivetrain);
    frc::SmartDashboard::PutData(std::string(nt::NetworkTable::BasenameKey(drivetrainTable->GetPath())) + "/Align Heading Controller", &container.alignToHub.HeadingController);
    frc::SmartDashboard::PutData(nt::NetworkTable::BasenameKey(hopperTable->GetPath()), &container.hopper);
    frc::SmartDashboard::PutData(nt::NetworkTable::BasenameKey(launcherTable->GetPath()), &container.launcher);
    frc::SmartDashboard::PutData(std::string(nt::NetworkTable::BasenameKey(intakePivotTable->GetPath())), &container.intakePivot);
    frc::SmartDashboard::PutData(std::string(nt::NetworkTable::BasenameKey(intakeRollerTable->GetPath())), &container.intakeRoller);
    
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
    frc::SmartDashboard::PutNumber("Manual Set Speed", container.launcherSetSpeed.value());
    
    turretCamPose.Set(frc::Pose3d{state.Pose}.TransformBy(VisionConstants::TurretCamera::kRobotToCamera));
    turretVisionPosePublisher.Set(container.turretVisionPose);
    turretVisionTargetsPublisher.Set(container.turretVisionTargets);
    backCam1Pose.Set(frc::Pose3d{state.Pose}.TransformBy(VisionConstants::BackCamera1::kRobotToCamera));
    backCam1VisionPosePublisher.Set(container.backCamera1Pose);
    backCam1VisionTargetsPublisher.Set(container.backCamera1Targets);

}