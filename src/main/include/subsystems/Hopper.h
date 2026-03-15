#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include "Constants.h"

using namespace ctre::phoenix6;

namespace HopperConstants
{
    inline constexpr double feederMotorSpeed = 1;
    inline constexpr double floorMotorSpeed = 1;
}

class Hopper : public frc2::SubsystemBase
{
public:
    Hopper();

    void FeedLauncher();
    void Eject();
    void StopMotors();
    frc2::CommandPtr StopMotorsCommand();
    frc2::CommandPtr FeedLauncherCommand();
    frc2::CommandPtr EjectCommand();

    void InitSendable(wpi::SendableBuilder &builder) override;

private:
    hardware::TalonFX feederMotor{RobotMap::Hopper::kFeederMotorID};
    hardware::TalonFX floorMotor{RobotMap::Hopper::kFloorMotorID};

    double feederMotorSpeed = HopperConstants::feederMotorSpeed;
    double floorMotorSpeed = HopperConstants::floorMotorSpeed;
};