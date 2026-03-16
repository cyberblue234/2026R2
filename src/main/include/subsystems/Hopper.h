#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include "Constants.h"

using namespace ctre::phoenix6;

namespace HopperConstants
{
    inline constexpr units::revolutions_per_minute_t kFeederMotorSpeed = 5000_rpm;
    inline constexpr double kFeederEjectSpeed = -0.85;
    inline constexpr double kP = 1.0;
    inline constexpr double kI = 0.0;
    inline constexpr double kD = 0.0;
    inline constexpr double kS = 0.0;
    inline constexpr double kV = 12.0 / frc::DCMotor::KrakenX60(1).freeSpeed.value();
    inline constexpr double kA = 0.0;
    inline constexpr double kFloorMotorSpeed = 0.83;
}

class Floor : public frc2::SubsystemBase
{
public:
    Floor();

    frc2::CommandPtr StopCommand();
    frc2::CommandPtr FeedCommand();
    frc2::CommandPtr EjectCommand();

    void InitSendable(wpi::SendableBuilder &builder) override;
private:
    hardware::TalonFX floorMotor{RobotMap::Hopper::kFloorMotorID};
    double floorMotorSpeed = HopperConstants::kFloorMotorSpeed;
};

class Feeder : public frc2::SubsystemBase
{
public:
    Feeder();

    frc2::CommandPtr StopCommand();
    frc2::CommandPtr FeedCommand();
    frc2::CommandPtr EjectCommand();

    void InitSendable(wpi::SendableBuilder &builder) override;

private:
    hardware::TalonFX feederMotor{RobotMap::Hopper::kFeederMotorID};
    controls::VelocityVoltage feederVelocityRequest{0_rpm};

    units::revolutions_per_minute_t feederMotorSpeed = HopperConstants::kFeederMotorSpeed;
    
};