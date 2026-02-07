#include "subsystems/Hopper.h"

Hopper::Hopper() {
    feederMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration feederMotorConfig;
    feederMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    feederMotorConfig.CurrentLimits.StatorCurrentLimit = 120_A;
    feederMotor.GetConfigurator().Apply(feederMotorConfig);
}

void Hopper::FeedLauncher() {
    feederMotor.Set(feederMotorSpeed);
    floorMotor.Set(floorMotorSpeed);
}

