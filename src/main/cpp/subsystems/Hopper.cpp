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

    SetDefaultCommand(StopMotorsCommand());
}

void Hopper::FeedLauncher()
{
    feederMotor.Set(feederMotorSpeed);
    floorMotor.Set(floorMotorSpeed);
}

void Hopper::StopMotors()
{
    feederMotor.StopMotor();
    floorMotor.StopMotor();
}

frc2::CommandPtr Hopper::StopMotorsCommand()
{
    return RunOnce([this] { StopMotors(); }).WithName("Stop Motors");
}

frc2::CommandPtr Hopper::FeedLauncherCommand()
{
    return Run([this]{ FeedLauncher(); }).WithName("Feed Launcher");
}

void Hopper::InitSendable(wpi::SendableBuilder &builder)
{
    builder.AddDoubleProperty("Feeder Set Speed", [this] { return feederMotorSpeed; }, [this] (double set) { feederMotorSpeed = set;});
    builder.AddDoubleProperty("Floor Set Speed", [this] { return floorMotorSpeed; }, [this] (double set) { floorMotorSpeed = set;});
    ADD_DEFAULT_COMMAND;
    ADD_CURRENT_COMMAND;
}