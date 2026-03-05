#include "Telemetry.h"

Telemetry::Telemetry(RobotContainer& container) : container(container)
{
    frc::SmartDashboard::PutData(std::string(nt::NetworkTable::BasenameKey(climbersTable->GetPath())) + "/1", &container.climber1);
    frc::SmartDashboard::PutData(std::string(nt::NetworkTable::BasenameKey(climbersTable->GetPath())) + "/2", &container.climber2);
    frc::SmartDashboard::PutData(nt::NetworkTable::BasenameKey(drivetrainTable->GetPath()), &container.drivetrain);
    frc::SmartDashboard::PutData("hopper", &container.hopper);
}

void Telemetry::UpdateTelemetry()
{
    swerve::impl::SwerveDrivetrainImpl::SwerveDriveState state = container.drivetrain.GetState();
    drivetrainPose.Set(state.Pose);
    moduleStates.Set(state.ModuleStates);
    moduleTargets.Set(state.ModuleTargets);
    modulePositions.Set(state.ModulePositions);
    chassisSpeeds.Set(state.Speeds);
}