#include "subsystems/Climber.h"

Climber::Climber() {
    climberMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration climberMotorConfig;
    climberMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climberMotorConfig.CurrentLimits.StatorCurrentLimit = 120_A;
    climberMotor.GetConfigurator().Apply(climberMotorConfig);
}

void Climber::FeedClimber() {
    climberMotor.Set(climberMotorSpeed);
    
}