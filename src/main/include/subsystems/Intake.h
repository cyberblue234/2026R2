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

    void SetIntakeOn();
    void SetIntakeOff();
    void IntakeToFloor();

     frc2::CommandPtr IntakeOnCommand();

private:
    hardware::TalonFX rollerMotor{RobotMap::Intake::kRollerMotorID};
    hardware::TalonFX pivotMotor{RobotMap::Intake::kPivotMotorID};

    controls::PositionVoltage pivotMotorPositionControl{0_tr};

    double rollerMotorSpeed = 0.5;
    double pivotMotorSpeed = 0.5;

};