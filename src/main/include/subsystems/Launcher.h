#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <frc/PWM.h>
#include <units/moment_of_inertia.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/simulation/FlywheelSim.h>
#include "Constants.h"

using namespace ctre::phoenix6;

namespace LauncherConstants
{
    constexpr units::kilogram_t kFuelMass = 0.5_lb;
    constexpr units::meter_t kFuelRadius = 5.91_in / 2;
    constexpr units::meter_t kCompression = 0.5_in;
    constexpr units::meter_t kEffectiveFuelRadius = kFuelRadius - kCompression;
    constexpr units::meter_t kShooterRadius = 2_in;
    constexpr units::kilogram_t kStealthWheelMass = 0.24_lb;
    constexpr units::kilogram_t kFlywheelMass = 0.47_lb;
    constexpr units::kilogram_t kShooterMass = kFlywheelMass + 3 * kStealthWheelMass;
    constexpr units::kilogram_square_meter_t kStealthWheelMOI = 0.634_lb * 1_in * 1_in; // Calculated with CAD
    constexpr units::kilogram_square_meter_t kFlywheelMOI = 1.159_lb * 1_in * 1_in; // Calculated with CAD
    constexpr units::kilogram_square_meter_t kShooterMOI = kFlywheelMOI + 3 * kStealthWheelMOI;
    constexpr units::kilogram_square_meter_t kFuelMOIInFlywheel{0.000339}; // Forced constant because WPILib wants to evaluate to 0
    
    constexpr double kLoss = 0.2; // 0% loss

    constexpr units::degree_t kDiscontinuityPointAngle = 300_deg;
    constexpr units::turn_t kMagnetOffset = 0_tr;

    constexpr frc::Translation3d kTurretOffset{10.5_in, 0_m, 22.5_in};
};

struct LauncherState
{
    units::degree_t pitch;
    units::radians_per_second_t omega;

    LauncherState() {}

    LauncherState(units::degree_t pitch, units::radians_per_second_t omega)
    {
        this->pitch = pitch;
        this->omega = omega;
    }
};


class Launcher : public frc2::SubsystemBase
{
public:
    Launcher();

    void SetLauncherPosition(double position);
    void SetLauncherAngle(units::degree_t angle);
    void SetLauncherSpeed(units::turns_per_second_t omega);
    void StopLauncher();
    bool IsLauncherSpeedWithinTolerance(units::radians_per_second_t tolerance = 0.5_tps);

    void Periodic() override
    {
        frc::SmartDashboard::PutNumber("Shooter Omega", GetLauncherOmega().value());
    }

    units::radians_per_second_t GetLauncherOmega()
    {
        return launcherMotor1.GetVelocity().GetValue();
    }

    std::function<units::angular_velocity::radians_per_second_t ()> GetLauncherOmegaSupplier()
    {
        return launcherMotor1.GetVelocity().AsSupplier();
    }

    units::degree_t GetLauncherAngle()
    {
        return deflectorCANcoder.GetAbsolutePosition().GetValue();
    }

    std::function<units::angle::degree_t ()> GetLauncherAngleAsSupplier()
    {
        return deflectorCANcoder.GetAbsolutePosition().AsSupplier();
    }

    frc2::CommandPtr ManualSetPosition(double position);
    frc2::CommandPtr StopMotorsCommand();
    frc2::CommandPtr LaunchCommand(std::function<LauncherState()> setState);

    void InitSendable(wpi::SendableBuilder& builder) override;

private:
    hardware::TalonFX launcherMotor1{RobotMap::Launcher::kLauncherMotor1ID};
    hardware::TalonFX launcherMotor2{RobotMap::Launcher::kLauncherMotor2ID};
    hardware::TalonFX launcherMotor3{RobotMap::Launcher::kLauncherMotor3ID};

    controls::VelocityVoltage launcherMotorVelocityControl{0_tps};
    LauncherState currentState;

    frc::PWM actuator1{RobotMap::Launcher::kActuator1ID};
    frc::PWM actuator2{RobotMap::Launcher::kActuator2ID};

    hardware::CANcoder deflectorCANcoder{RobotMap::Launcher::kDeflectorCANcoderID};

    frc::sim::FlywheelSim launcherSim
    {
        frc::LinearSystemId::FlywheelSystem(frc::DCMotor::KrakenX60(1), LauncherConstants::kShooterMOI, 1),
        frc::DCMotor::KrakenX60(1)
    };
};