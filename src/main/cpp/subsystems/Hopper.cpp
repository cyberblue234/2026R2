#include "subsystems/Hopper.h"

Hopper::Hopper() {
    feederMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration feederMotorConfig;
    feederMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    feederMotorConfig.CurrentLimits.StatorCurrentLimit = 120_A;
    feederMotor.GetConfigurator().Apply(feederMotorConfig);
}

void Hopper::FeedLauncherOn() {
    feederMotor.Set(feederMotorSpeed);
    floorMotor.Set(floorMotorSpeed);
}

void Hopper::FeedLauncherOff() {
    feederMotor.StopMotor();
    floorMotor.StopMotor();
}

frc2::CommandPtr Hopper::FeedLauncherCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return Run([this] 
    {
        FeedLauncherOn();
    }).FinallyDo([this] 
    {
        FeedLauncherOff();
    });
}