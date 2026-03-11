#pragma once

#include "Constants.h"
#include "RobotContainer.h"

class Telemetry
{
public:
    Telemetry(RobotContainer& container);
    
    void UpdateTelemetry();
private:
    RobotContainer& container;

    std::shared_ptr<nt::NetworkTable> climbersTable = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetSubTable("Climbers");

    std::shared_ptr<nt::NetworkTable> drivetrainTable = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetSubTable("Drivetrain");
    nt::StructPublisher<frc::Pose2d> drivetrainPose = drivetrainTable->GetStructTopic<frc::Pose2d>("Pose").Publish();
    nt::StructArrayPublisher<frc::SwerveModuleState> moduleStates = drivetrainTable->GetStructArrayTopic<frc::SwerveModuleState>("Module States").Publish();
    nt::StructArrayPublisher<frc::SwerveModuleState> moduleTargets = drivetrainTable->GetStructArrayTopic<frc::SwerveModuleState>("Module Targets").Publish();
    nt::StructArrayPublisher<frc::SwerveModulePosition> modulePositions = drivetrainTable->GetStructArrayTopic<frc::SwerveModulePosition>("Module Positions").Publish();
    nt::StructPublisher<frc::ChassisSpeeds> chassisSpeeds = drivetrainTable->GetStructTopic<frc::ChassisSpeeds>("Chassis Speeds").Publish();

    std::shared_ptr<nt::NetworkTable> hopperTable = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetSubTable("Hopper");
    std::shared_ptr<nt::NetworkTable> launcherTable = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetSubTable("Launcher");
    std::shared_ptr<nt::NetworkTable> intakePivotTable = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetSubTable("Intake Pivot");
    nt::StructPublisher<frc::Pose3d> intakePose = intakePivotTable->GetStructTopic<frc::Pose3d>("Pose").Publish();
    std::shared_ptr<nt::NetworkTable> intakeRollerTable = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetSubTable("Intake Roller");

    std::shared_ptr<nt::NetworkTable> genericTable = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetSubTable("Generic");
    nt::StringPublisher targetPublisher = genericTable->GetStringTopic("Target").Publish();

    std::shared_ptr<nt::NetworkTable> visionTable = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetSubTable("Vision");
    nt::StructPublisher<frc::Pose3d> turretCamPose = visionTable->GetSubTable("Turret")->GetStructTopic<frc::Pose3d>("Camera Pose").Publish();
    nt::StructPublisher<frc::Pose3d> turretVisionPosePublisher = visionTable->GetSubTable("Turret")->GetStructTopic<frc::Pose3d>("Estimated Pose").Publish();
    nt::StructArrayPublisher<frc::Pose3d> turretVisionTargetsPublisher = visionTable->GetSubTable("Turret")->GetStructArrayTopic<frc::Pose3d>("Targets").Publish();
    nt::StructPublisher<frc::Pose3d> backCam1Pose = visionTable->GetSubTable("Back Camera 1")->GetStructTopic<frc::Pose3d>("Camera Pose").Publish();
    nt::StructPublisher<frc::Pose3d> backCam1VisionPosePublisher = visionTable->GetSubTable("Back Camera 1")->GetStructTopic<frc::Pose3d>("Estimated Pose").Publish();
    nt::StructArrayPublisher<frc::Pose3d> backCam1VisionTargetsPublisher = visionTable->GetSubTable("Back Camera 1")->GetStructArrayTopic<frc::Pose3d>("Targets").Publish();
};
