#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <frc/simulation/SingleJointedArmSim.h>
#include "Constants.h"

using namespace ctre::phoenix6;

namespace IntakeConstants
{
    constexpr units::degree_t kGroundPosition = 110_deg;
    constexpr units::degree_t kHomePosition = 0_deg;

    constexpr double kIntakeRollerSpeed = 0.85;
    constexpr double kEjectRollerSpeed = -1.0;
    
    constexpr double kMotorToCANcoderRatio = 25;
    constexpr double kCANcoderToPivotRatio = 1.75;
    constexpr double kMotorToPivotRatio = kMotorToCANcoderRatio * kCANcoderToPivotRatio;

    inline constexpr double kManualSpeed = 0.2;
}

class IntakeRoller : public frc2::SubsystemBase
{
public:
    IntakeRoller();

    void SetMotor(double speed);
    void StopMotor();
    
    frc2::CommandPtr StopMotorCommand();
    frc2::CommandPtr IntakeCommand();
    frc2::CommandPtr EjectCommand();

    void InitSendable(wpi::SendableBuilder &builder) override;
private:
    hardware::TalonFX rollerMotor{RobotMap::Intake::kRollerMotorID};

    double motorSpeed = IntakeConstants::kIntakeRollerSpeed;
};



class IntakePivot : public frc2::SubsystemBase
{
private:
    hardware::TalonFX pivotMotor{RobotMap::Intake::kPivotMotorID};

    controls::VoltageOut positionVoltageOut{0_V};
    controls::DutyCycleOut pivotMotorSpeedControl{0};
    
    frc::sim::SingleJointedArmSim intakeSim
    {
        frc::LinearSystemId::SingleJointedArmSystem(frc::DCMotor::KrakenX60(1), 0.1_kg_sq_m, IntakeConstants::kMotorToPivotRatio),
        frc::DCMotor::KrakenX60(1),
        IntakeConstants::kMotorToPivotRatio,
        295.7_mm,
        IntakeConstants::kHomePosition,
        IntakeConstants::kGroundPosition,
        false,
        IntakeConstants::kHomePosition
    };
    
public:
    IntakePivot();
    
    void StopMotor();
    
    frc2::CommandPtr StopMotorCommand();
    frc2::CommandPtr SetMotorToBrakeCommand();
    frc2::CommandPtr SetSpeedCommand(double speed);
    frc2::CommandPtr SetPositionToGroundCommand();
    frc2::CommandPtr SetPositionToHomeCommand();
    frc2::CommandPtr BounceCommand();
    
    void SimulationPeriodic() override;

    units::degree_t GetAngle()
    {
        return pivotMotor.GetPosition().GetValue() / IntakeConstants::kMotorToPivotRatio;
    }
    
    void InitSendable(wpi::SendableBuilder &builder) override;
};