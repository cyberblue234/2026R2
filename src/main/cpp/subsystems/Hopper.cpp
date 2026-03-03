#include "subsystems/Hopper.h"

Hopper::Hopper()
{
    // Basic Configs that both motors will have
    configs::TalonFXConfiguration motorConfigs;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 120_A;
    motorConfigs.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    feederMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration feederMotorConfig{motorConfigs};
    feederMotorConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    feederMotor.GetConfigurator().Apply(feederMotorConfig);

    floorMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration floorMotorConfig{motorConfigs};
    floorMotorConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    floorMotor.GetConfigurator().Apply(floorMotorConfig);
}

void Hopper::FeedLauncher()
{
    feederMotor.Set(HopperConstants::feederMotorSpeed);
    floorMotor.Set(HopperConstants::floorMotorSpeed);
}

void Hopper::StopMotors()
{
    feederMotor.StopMotor();
    floorMotor.StopMotor();
}

frc2::CommandPtr Hopper::FeedLauncherCommand()
{
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return Run([this]
               { FeedLauncher(); })
        .FinallyDo([this]
                   { StopMotors(); });
}