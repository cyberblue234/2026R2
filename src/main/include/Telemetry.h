#pragma once

#include "Constants.h"
#include "RobotContainer.h"

namespace TelemetryConstants
{
    inline const std::string climbersTableName = "Climbers";
    inline const std::string drivetrainTableName = "Drivetrain";
    inline const std::string hopperTableName = "Hopper";
    inline const std::string floorTableName = "Floor";
    inline const std::string feederTableName = "Feeder";
    inline const std::string launcherTableName = "Launcher";
    inline const std::string intakePivotTableName = "Intake Pivot";
    inline const std::string intakeRollerTableName = "Intake Roller";
    inline const std::string genericTableName = "Generic";
}

using namespace TelemetryConstants;

class Telemetry
{
public:
    Telemetry(RobotContainer& container);
    
    void UpdateTelemetry();
private:
    RobotContainer& container;

    std::shared_ptr<nt::NetworkTable> smartdashboardTable = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard");
    std::shared_ptr<nt::NetworkTable> climbersTable = smartdashboardTable->GetSubTable(climbersTableName);

    std::shared_ptr<nt::NetworkTable> drivetrainTable = smartdashboardTable->GetSubTable(drivetrainTableName);
    nt::StructPublisher<frc::Pose2d> drivetrainPose = drivetrainTable->GetStructTopic<frc::Pose2d>("Pose").Publish();
    nt::StructArrayPublisher<frc::SwerveModuleState> moduleStates = drivetrainTable->GetStructArrayTopic<frc::SwerveModuleState>("Module States").Publish();
    nt::StructArrayPublisher<frc::SwerveModuleState> moduleTargets = drivetrainTable->GetStructArrayTopic<frc::SwerveModuleState>("Module Targets").Publish();
    nt::StructArrayPublisher<frc::SwerveModulePosition> modulePositions = drivetrainTable->GetStructArrayTopic<frc::SwerveModulePosition>("Module Positions").Publish();
    nt::StructPublisher<frc::ChassisSpeeds> chassisSpeeds = drivetrainTable->GetStructTopic<frc::ChassisSpeeds>("Chassis Speeds").Publish();

    std::shared_ptr<nt::NetworkTable> hopperTable = smartdashboardTable->GetSubTable(hopperTableName);
    std::shared_ptr<nt::NetworkTable> floorTable = hopperTable->GetSubTable(feederTableName);
    std::shared_ptr<nt::NetworkTable> feederTable = hopperTable->GetSubTable(floorTableName);
    std::shared_ptr<nt::NetworkTable> launcherTable = smartdashboardTable->GetSubTable(launcherTableName);
    std::shared_ptr<nt::NetworkTable> intakePivotTable = smartdashboardTable->GetSubTable(intakePivotTableName);
    nt::StructPublisher<frc::Pose3d> intakePose = intakePivotTable->GetStructTopic<frc::Pose3d>("Pose").Publish();
    std::shared_ptr<nt::NetworkTable> intakeRollerTable = smartdashboardTable->GetSubTable(intakeRollerTableName);

    std::shared_ptr<nt::NetworkTable> genericTable = smartdashboardTable->GetSubTable(genericTableName);
    nt::StringPublisher targetPublisher = genericTable->GetStringTopic("Target").Publish();

    std::shared_ptr<nt::NetworkTable> visionTable = smartdashboardTable->GetSubTable("Vision");
    nt::StructPublisher<frc::Pose3d> turretCamPose = visionTable->GetSubTable("Turret")->GetStructTopic<frc::Pose3d>("Camera Pose").Publish();
    nt::StructPublisher<frc::Pose3d> turretVisionPosePublisher = visionTable->GetSubTable("Turret")->GetStructTopic<frc::Pose3d>("Estimated Pose").Publish();
    nt::StructArrayPublisher<frc::Pose3d> turretVisionTargetsPublisher = visionTable->GetSubTable("Turret")->GetStructArrayTopic<frc::Pose3d>("Targets").Publish();
    nt::StructPublisher<frc::Pose3d> backCam1Pose = visionTable->GetSubTable("Back Camera 1")->GetStructTopic<frc::Pose3d>("Camera Pose").Publish();
    nt::StructPublisher<frc::Pose3d> backCam1VisionPosePublisher = visionTable->GetSubTable("Back Camera 1")->GetStructTopic<frc::Pose3d>("Estimated Pose").Publish();
    nt::StructArrayPublisher<frc::Pose3d> backCam1VisionTargetsPublisher = visionTable->GetSubTable("Back Camera 1")->GetStructArrayTopic<frc::Pose3d>("Targets").Publish();
};
