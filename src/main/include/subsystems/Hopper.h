#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include "Constants.h"

using namespace ctre::phoenix6;


class Hopper : public frc2::SubsystemBase {
public:

    Hopper();

    void FeedLauncher();


private:
    hardware::TalonFX feederMotor{RobotMap::Hopper::kFeederMotorID};
    hardware::TalonFX floorMotor{RobotMap::Hopper::kFloorMotorID};

    double feederMotorSpeed = 0.5;
    double floorMotorSpeed = 0.5;

};