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
    pivotMotor.GetConfigurator().Apply(pivotMotorConfig);
}

void Intake::FeedRoller() {
    rollerMotor.Set(rollerMotorSpeed);  
}

void Intake::FeedPivot() {
    pivotMotor.Set(pivotMotorSpeed);
}

