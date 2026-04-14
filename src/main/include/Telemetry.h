#pragma once

#include "Constants.h"
#include "frc/smartdashboard/Field2d.h"
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
    inline const std::string visionTableName = "Vision";
    inline const std::string turretCameraTableName = "Turret";
    inline const std::string backCameraLeftTableName = "Back Left";
    inline const std::string backCameraRightTableName = "Back Right";
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
    frc::Field2d field;
    nt::StructArrayPublisher<frc::SwerveModuleState> moduleStates = drivetrainTable->GetStructArrayTopic<frc::SwerveModuleState>("Module States").Publish();
    nt::StructArrayPublisher<frc::SwerveModuleState> moduleTargets = drivetrainTable->GetStructArrayTopic<frc::SwerveModuleState>("Module Targets").Publish();
    nt::StructArrayPublisher<frc::SwerveModulePosition> modulePositions = drivetrainTable->GetStructArrayTopic<frc::SwerveModulePosition>("Module Positions").Publish();
    nt::StructPublisher<frc::ChassisSpeeds> chassisSpeeds = drivetrainTable->GetStructTopic<frc::ChassisSpeeds>("Chassis Speeds").Publish();
    nt::StructPublisher<frc::ChassisSpeeds> autoChassisSpeeds = drivetrainTable->GetStructTopic<frc::ChassisSpeeds>("Auto Chassis Speeds").Publish();

    std::shared_ptr<nt::NetworkTable> hopperTable = smartdashboardTable->GetSubTable(hopperTableName);
    std::shared_ptr<nt::NetworkTable> floorTable = hopperTable->GetSubTable(feederTableName);
    std::shared_ptr<nt::NetworkTable> feederTable = hopperTable->GetSubTable(floorTableName);
    std::shared_ptr<nt::NetworkTable> launcherTable = smartdashboardTable->GetSubTable(launcherTableName);
    std::shared_ptr<nt::NetworkTable> intakePivotTable = smartdashboardTable->GetSubTable(intakePivotTableName);
    nt::StructPublisher<frc::Pose3d> intakePose = intakePivotTable->GetStructTopic<frc::Pose3d>("Pose").Publish();
    std::shared_ptr<nt::NetworkTable> intakeRollerTable = smartdashboardTable->GetSubTable(intakeRollerTableName);

    std::shared_ptr<nt::NetworkTable> genericTable = smartdashboardTable->GetSubTable(genericTableName);
    nt::StringPublisher targetPublisher = genericTable->GetStringTopic("Target").Publish();
    nt::DoublePublisher manualSetSpeedPublisher = genericTable->GetDoubleTopic("Manual Set Speed").Publish();
    nt::BooleanPublisher alignmentWithinTolerance = genericTable->GetBooleanTopic("Is Alignment Within Tolerances").Publish();
    nt::BooleanPublisher visionEnabledPublisher = genericTable->GetBooleanTopic("Vision Enabled").Publish();

    nt::DoublePublisher robotVoltagePublisher = genericTable->GetDoubleTopic("Robot Voltage").Publish();

    std::shared_ptr<nt::NetworkTable> visionTable = smartdashboardTable->GetSubTable(visionTableName);
    nt::StructPublisher<frc::Pose3d> turretVisionPosePublisher = visionTable->GetSubTable(turretCameraTableName)->GetStructTopic<frc::Pose3d>("Estimated Pose").Publish();
    nt::StructPublisher<frc::Pose3d> backCamLeftVisionPosePublisher = visionTable->GetSubTable(backCameraLeftTableName)->GetStructTopic<frc::Pose3d>("Estimated Pose").Publish();
    nt::StructPublisher<frc::Pose3d> backCamRightVisionPosePublisher = visionTable->GetSubTable(backCameraRightTableName)->GetStructTopic<frc::Pose3d>("Estimated Pose").Publish();
};