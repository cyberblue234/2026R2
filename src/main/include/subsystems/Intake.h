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
    constexpr units::degree_t kBouncePosition = 80_deg;

    constexpr units::degree_t kPivotTolerance = 3_deg;

    constexpr double kIntakeRollerSpeed = 0.85;
    constexpr double kEjectRollerSpeed = -1.0;

    constexpr double kP = 0.0;
    constexpr double kI = 0.0;
    constexpr double kD = 0.0;
    constexpr double kS = 0.0;
    constexpr double kG = 0.0;
    constexpr double kV = 0.0;
    constexpr double kA = 0.0;
    constexpr units::degree_t kGravityArmPositionOffset = 0_deg;

    constexpr units::degrees_per_second_t kMaxVelocity = 360_deg_per_s;
    constexpr units::degrees_per_second_squared_t kMaxAcceleration = 720_deg_per_s_sq;
    constexpr units::degrees_per_second_cubed_t kMaxJerk = 0_deg_per_s_cu;

    constexpr units::degree_t kDiscontinuityPointAngle = 300_deg;
    constexpr units::turn_t kMagnetOffset = 0.16_tr;
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
    hardware::CANcoder pivotCancoder{RobotMap::Intake::kPivotCancoderID};

    // controls::PositionVoltage pivotMotorPositionControl{0_tr};
    controls::MotionMagicVoltage pivotMotorPositionControl{0_tr};
    controls::VoltageOut positionVoltageOut{0_V};
    controls::DutyCycleOut pivotMotorSpeedControl{0};
    
    units::degree_t groundPosition = IntakeConstants::kGroundPosition;
    units::degree_t homePosition = IntakeConstants::kHomePosition;
    units::degree_t bouncePosition = IntakeConstants::kBouncePosition;
    units::degree_t tolerance = IntakeConstants::kPivotTolerance;
    
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
    
    void SetPosition(units::degree_t angle);
    void SetPositionToGround();
    void SetPositionToHome();
    void SetPositionToBounce();
    void StopMotor();
    bool IsWithinTolerance();
    
    units::degree_t GetAngle()
    {
        return pivotCancoder.GetAbsolutePosition().GetValue() / IntakeConstants::kCANcoderToPivotRatio;
    }
    
    frc2::CommandPtr StopMotorCommand();
    frc2::CommandPtr SetMotorToBrakeCommand();
    frc2::CommandPtr SetSpeedCommand(double speed);
    frc2::CommandPtr SetPositionCommand(std::function<units::degree_t()> angle);
    frc2::CommandPtr SetPositionToGroundCommand();
    frc2::CommandPtr SetPositionToHomeCommand();
    frc2::CommandPtr SetPositionToBounceCommand();
    frc2::CommandPtr BounceCommand();
    
    void SimulationPeriodic() override;
    
    void InitSendable(wpi::SendableBuilder &builder) override;

    frc::PIDController positionController{IntakeConstants::kP, IntakeConstants::kI, IntakeConstants::kD};
};