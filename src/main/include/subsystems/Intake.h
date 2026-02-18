#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include "Constants.h"

using namespace ctre::phoenix6;


class Intake : public frc2::SubsystemBase {
public:

    Intake();

    void SetRollerMotor(double speed);
    void StopRollerMotor();
    void SetPosition(units::degree_t angle);
    void SetPositionToGround();
    void SetPositionToHome();
    void SetPositionToBounce();
    void StopPivotMotor();
    bool IsPivotWithinTolerance();

    frc2::CommandPtr ManualIntakeCommand();
    frc2::CommandPtr SetPositionToGroundCommand();
    frc2::CommandPtr SetPositionToHomeCommand();
    frc2::CommandPtr IntakeFuelCommand();
    frc2::CommandPtr BounceCommand();

private:
    hardware::TalonFX rollerMotor{RobotMap::Intake::kRollerMotorID};
    hardware::TalonFX pivotMotor{RobotMap::Intake::kPivotMotorID};

    hardware::CANcoder pivotCancoder{RobotMap::Intake::kPivotCancoderID};

    controls::PositionVoltage pivotMotorPositionControl{0_tr};

    bool setToGround;
};