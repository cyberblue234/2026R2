#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include "Constants.h"

using namespace ctre::phoenix6;

class IntakeRoller : public frc2::SubsystemBase
{
public:
    IntakeRoller();

    void SetRollerMotor(double speed);
    void StopRollerMotor();
    
    frc2::CommandPtr StartIntakeCommand();
    frc2::CommandPtr EjectCommand();
private:
    hardware::TalonFX rollerMotor{RobotMap::Intake::kRollerMotorID};
};

class IntakePivot : public frc2::SubsystemBase
{
public:
    IntakePivot();

    void SetPosition(units::degree_t angle);
    void SetPositionToGround();
    void SetPositionToHome();
    void SetPositionToBounce();
    void StopPivotMotor();
    bool IsPivotWithinTolerance();

    frc2::CommandPtr SetPositionToGroundCommand();
    frc2::CommandPtr SetPositionToHomeCommand();
    frc2::CommandPtr SetPositionToBounceCommand();
    frc2::CommandPtr BounceCommand();

private:
    hardware::TalonFX pivotMotor{RobotMap::Intake::kPivotMotorID};

    hardware::CANcoder pivotCancoder{RobotMap::Intake::kPivotCancoderID};

    controls::PositionVoltage pivotMotorPositionControl{0_tr};

    bool setToGround;
};

namespace IntakeConstants
{
    constexpr units::degree_t kGroundPosition = 90_deg;
    constexpr units::degree_t kHomePosition = 0_deg;
    constexpr units::degree_t kBouncePosition = 45_deg;

    constexpr units::degree_t kPivotTolerance = 5_deg;

    constexpr double kIntakeRollerSpeed = 1.0;
    constexpr double kEjectRollerSpeed = -1.0;

    constexpr units::degree_t kDiscontinuityPointAngle = 300_deg;
    constexpr units::turn_t kMagnetOffset = 0_tr;
    constexpr double kPivotToCANcoderRatio = 50;
}