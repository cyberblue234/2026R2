#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <frc/PWM.h>
#include <units/moment_of_inertia.h>
#include "Constants.h"

using namespace ctre::phoenix6;

class Launcher : public frc2::SubsystemBase
{
public:
    Launcher();

    void SetLauncherPosition(double position);
    void SetLauncherSpeed(units::turns_per_second_t omega);
    void StopLauncher();
    bool IsLauncherSpeedWithinTolerance();

    frc2::CommandPtr RunLauncherCommand(units::turns_per_second_t omega);

private:
    hardware::TalonFX launcherMotor1{RobotMap::Launcher::kLauncherMotor1ID};
    hardware::TalonFX launcherMotor2{RobotMap::Launcher::kLauncherMotor2ID};
    hardware::TalonFX launcherMotor3{RobotMap::Launcher::kLauncherMotor3ID};

    controls::VelocityVoltage launcherMotorVelocityControl{0_tps};

    frc::PWM actuator1{RobotMap::Launcher::kActuator1ID};
    frc::PWM actuator2{RobotMap::Launcher::kActuator2ID};
};

struct LauncherState
{
    units::degree_t pitch;
    units::radians_per_second_t omega;
};

namespace LauncherConstants
{
    constexpr units::meter_t kFlywheelRadius = 2_in; // Need to check
    constexpr units::meter_t kFuelRadius = 5.91_in / 2;
    constexpr units::meter_t kCompression = 1_in;
    constexpr units::meter_t kEffectiveFuelRadius = kFuelRadius - kCompression / 2;
    constexpr units::kilogram_t kFlywheelMass = 0.98_lb;
    // constexpr units::kilogram_square_meter_t = units::kilogram_square_meter_t{2};
    constexpr units::kilogram_t kFuelMass = 0.5_lb;
    constexpr double kLoss = 0; // 0% loss
};