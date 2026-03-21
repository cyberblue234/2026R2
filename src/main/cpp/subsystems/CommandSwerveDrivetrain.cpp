#include "subsystems/CommandSwerveDrivetrain.h"
#include <frc/RobotController.h>

void CommandSwerveDrivetrain::Periodic()
{
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
        auto const allianceColor = frc::DriverStation::GetAlliance();
        if (allianceColor) {
            SetOperatorPerspectiveForward(
                *allianceColor == frc::DriverStation::Alliance::kRed
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        }
    }

    frc::Pose2d pose = GetState().Pose;
    units::second_t currentTime = utils::GetSystemTime();
    units::second_t deltaTime = currentTime - lastTime;
    units::meter_t currentX = pose.X();
    vX = (currentX - lastX) / deltaTime;
    units::meter_t currentY = pose.Y();
    vY = (currentY - lastY) / deltaTime;
    units::radian_t currentYaw = pose.Rotation().Radians();
    vYaw = (currentYaw - lastYaw) / deltaTime;
    lastX = currentX;
    lastY = currentY;
    lastYaw = currentYaw;
    lastTime = currentTime;
}

void CommandSwerveDrivetrain::StartSimThread()
{
    m_lastSimTime = utils::GetCurrentTime();
    m_simNotifier = std::make_unique<frc::Notifier>([this] {
        units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
    });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

void CommandSwerveDrivetrain::InitSendable(wpi::SendableBuilder &builder)
{
    builder.AddDoubleArrayProperty("Drive Supply Currents", [this] 
    { 
        std::vector<double> supplyCurrents;
        for (auto const& motors : GetModules())
        {
            supplyCurrents.push_back(motors->GetDriveMotor().GetSupplyCurrent().GetValueAsDouble());
        }
        return supplyCurrents; 
    }, nullptr);
    builder.AddDoubleArrayProperty("Steer Supply Currents", [this] 
    { 
        std::vector<double> supplyCurrents;
        for (auto const& motors : GetModules())
        {
            supplyCurrents.push_back(motors->GetSteerMotor().GetSupplyCurrent().GetValueAsDouble());
        }
        return supplyCurrents; 
    }, nullptr);
    builder.AddDoubleArrayProperty("Drive Stator Currents", [this] 
    { 
        std::vector<double> statorCurrents;
        for (auto const& motors : GetModules())
        {
            statorCurrents.push_back(motors->GetDriveMotor().GetStatorCurrent().GetValueAsDouble());
        }
        return statorCurrents; 
    }, nullptr);
    builder.AddDoubleArrayProperty("Steer Stator Currents", [this] 
    { 
        std::vector<double> statorCurrents;
        for (auto const& motors : GetModules())
        {
            statorCurrents.push_back(motors->GetSteerMotor().GetStatorCurrent().GetValueAsDouble());
        }
        return statorCurrents; 
    }, nullptr);
    
    ADD_DEFAULT_COMMAND;
    ADD_CURRENT_COMMAND;
}