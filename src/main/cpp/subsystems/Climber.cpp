#include "subsystems/Climber.h"

Climber::Climber()
{
    rev::spark::SparkBaseConfig config;
    config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    config.SmartCurrentLimit(60);
    config.Inverted(false);
    motor.Configure(config, rev::ResetMode::kResetSafeParameters,
                    rev::PersistMode::kNoPersistParameters);

    climberLimitSwitchTrigger.Debounce(60_ms).OnTrue(ResetClimberEncoderCommand());

    SetDefaultCommand(StopClimberCommand());
}

void Climber::ExtendClimber()
{
    motor.Set(motorSpeed);
}

void Climber::RetractClimber()
{
    motor.Set(-motorSpeed);
}

void Climber::StopClimber()
{
    motor.StopMotor();
}

frc2::CommandPtr Climber::ResetClimberEncoderCommand()
{
    return frc2::cmd::RunOnce([this]
        {
            motor.GetEncoder().SetPosition(0);
            isRegistered = true; 
        }
    ).WithName("Reset Climber Encoder");
}

frc2::CommandPtr Climber::StopClimberCommand()
{
    return RunOnce([this]{ StopClimber(); }).WithName("Stop Climber");
}

frc2::CommandPtr Climber::ExtendClimberCommand()
{
    return Run([this] { ExtendClimber(); }).WithName("Extend Climber");
}

frc2::CommandPtr Climber::ExtendClimberWithLimitCommand()
{
    return ExtendClimberCommand()
        .OnlyIf([this] { return isRegistered; })
        .Until([this] { return GetClimberPosition() > maxPosition;})
        .WithName("Extend Climber With Limits");
}

frc2::CommandPtr Climber::RetractClimberCommand()
{
    return Run([this] { RetractClimber(); }).WithName("Retract Climber");
}

frc2::CommandPtr Climber::RetractClimberWithLimitCommand()
{
    return RetractClimberCommand().Unless([this] { return GetLimitSwitches(); })
    .Until([this] { return GetLimitSwitches(); })
    .WithName("Retract Climber With Limits");
}

void Climber::InitSendable(wpi::SendableBuilder &builder)
{
    builder.AddDoubleProperty("Position (tr)", [this] { return GetClimberPosition().value(); }, nullptr);
    builder.AddBooleanProperty("Limit Switch 1", [this] { return climberLimitSwitch1.Get(); }, nullptr);
    builder.AddBooleanProperty("Limit Switch 2", [this] { return climberLimitSwitch2.Get(); }, nullptr);
    builder.AddDoubleProperty("Motor Speed", [this] { return motorSpeed; }, [this] (double set) { motorSpeed = set;});
    builder.AddDoubleProperty("Max Position", [this] { return maxPosition.value(); }, [this] (double set) { maxPosition = units::turn_t{set};});
    ADD_DEFAULT_COMMAND;
    ADD_CURRENT_COMMAND;
}