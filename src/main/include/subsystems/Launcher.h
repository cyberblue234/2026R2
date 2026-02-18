#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <frc/PWM.h>
#include "Constants.h"

using namespace ctre::phoenix6;

class Launcher : public frc2::SubsystemBase
{
public:
    Launcher();

    void FeedLauncher();
    void SetLauncherPosition();
    void SetLauncherSpeed(units::turns_per_second_t omega);
    void StopLauncher();

    frc2::CommandPtr RunLauncherCommand(units::turns_per_second_t omega);

private:
    hardware::TalonFX launcherMotor1{RobotMap::Launcher::kLauncherMotor1ID};
    hardware::TalonFX launcherMotor2{RobotMap::Launcher::kLauncherMotor2ID};
    hardware::TalonFX launcherMotor3{RobotMap::Launcher::kLauncherMotor3ID};

    controls::VelocityVoltage launcherMotorVelocityControl{0_tps};

    frc::PWM actuator1{RobotMap::Launcher::kActuator1ID};
    frc::PWM actuator2{RobotMap::Launcher::kActuator2ID};

    double launcherMotorSpeed = 0.5;

    double actuatorPosition = 0.5;
};