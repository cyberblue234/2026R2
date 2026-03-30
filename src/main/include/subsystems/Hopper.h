#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include "Constants.h"

using namespace ctre::phoenix6;

namespace HopperConstants
{
    namespace Feeder
    {
        inline constexpr units::revolutions_per_minute_t kMotorSpeed = 5000_rpm;
        inline constexpr double kEjectSpeed = -0.85;
        inline constexpr double kP = 0.0;
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.0;
        inline constexpr double kS = 0.0;
        inline constexpr double kV = 0.115;
        inline constexpr double kA = 0.0;
    }
    namespace Floor
    {
        inline constexpr units::revolutions_per_minute_t kMotorSpeed = 0.8 * Feeder::kMotorSpeed;
        inline constexpr double kEjectSpeed = -0.6;
        inline constexpr double kP = 0.0;
        inline constexpr double kI = 0.0;
        inline constexpr double kD = 0.0;
        inline constexpr double kS = 0.0;
        inline constexpr double kV = 0.115;
        inline constexpr double kA = 0.0;
    }
}
class Floor : public frc2::SubsystemBase
{
public:
    Floor();

    frc2::CommandPtr StopCommand();
    frc2::CommandPtr FeedCommand();
    frc2::CommandPtr EjectCommand();

    bool IsFloorRunning() { return units::math::abs(floorMotor.GetVelocity().GetValue()) > 0.15_tps;  }

    void InitSendable(wpi::SendableBuilder &builder) override;
private:
    hardware::TalonFX floorMotor{RobotMap::Hopper::kFloorMotorID};
    controls::VelocityVoltage floorVelocityRequest{0_rpm};

    units::revolutions_per_minute_t floorMotorSpeed = HopperConstants::Floor::kMotorSpeed;
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

    units::revolutions_per_minute_t feederMotorSpeed = HopperConstants::Feeder::kMotorSpeed;
    
};