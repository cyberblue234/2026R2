#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include "Constants.h"

using namespace ctre::phoenix6;


class Intake : public frc2::SubsystemBase {
public:

    Intake();

    void FeedRoller();
    void FeedPivot();

private:
    hardware::TalonFX rollerMotor{RobotMap::Intake::kRollerMotorID};
    hardware::TalonFX pivotMotor{RobotMap::Intake::kPivotMotorID};

    double rollerMotorSpeed = 0.5;
    double pivotMotorSpeed = 0.5;

};