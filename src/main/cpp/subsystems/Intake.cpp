#include "subsystems/Intake.h"

Intake::Intake() {
    rollerMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration rollerMotorConfig;
    rollerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerMotorConfig.CurrentLimits.StatorCurrentLimit = 120_A;
    rollerMotor.GetConfigurator().Apply(rollerMotorConfig);

    pivotMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration pivotMotorConfig;
    pivotMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotMotorConfig.CurrentLimits.StatorCurrentLimit = 120_A;

    pivotMotorConfig.Slot0.kP = 2;
    pivotMotorConfig.Slot0.kI = 0.0;
    pivotMotorConfig.Slot0.kD = 0.1;
    
    pivotMotor.GetConfigurator().Apply(pivotMotorConfig);
}

void Intake::SetIntakeOn() {
    rollerMotor.Set(rollerMotorSpeed); 
}

void Intake::SetIntakeOff() {
    rollerMotor.StopMotor();
}

void Intake::IntakeToFloor() {
    pivotMotor.SetControl(pivotMotorPositionControl.WithPosition(90_deg));
}

frc2::CommandPtr Intake::IntakeOnCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] 
    {
        SetIntakeOn();
    }).FinallyDo([this] 
    {
        SetIntakeOff();
    });
}

