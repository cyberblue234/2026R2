#include "subsystems/Climber.h"

Climber::Climber(int motorID, int limitSwitchID, bool inverted) : climberMotor(motorID), climberLimitSwitch(limitSwitchID)
{
    climberMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration climberMotorConfig;
    climberMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climberMotorConfig.CurrentLimits.StatorCurrentLimit = 120_A;
    climberMotorConfig.MotorOutput.Inverted = inverted ? signals::InvertedValue::Clockwise_Positive : signals::InvertedValue::CounterClockwise_Positive;
    climberMotorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    climberMotor.GetConfigurator().Apply(climberMotorConfig);

    climberLimitSwitchTrigger.Debounce(60_ms).OnTrue(StopClimberCommand().AndThen(ResetClimberEncoderCommand()));

    SetDefaultCommand(StopClimberCommand());
}

void Climber::ExtendClimber()
{
    climberMotor.Set(motorSpeed);
}

void Climber::RetractClimber()
{
    climberMotor.Set(-motorSpeed);
}

void Climber::StopClimber()
{
    climberMotor.StopMotor();
}

frc2::CommandPtr Climber::ResetClimberEncoderCommand()
{
    return RunOnce([this]
        {
            climberMotor.SetPosition(0_tr);
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

void Climber::InitSendable(wpi::SendableBuilder &builder)
{
    builder.AddDoubleProperty("Position (tr)", [this] { return GetClimberPosition().value(); }, nullptr);
    builder.AddBooleanProperty("Limit Switch", [this] { return climberLimitSwitch.Get(); }, nullptr);
    builder.AddDoubleProperty("Motor Speed", [this] { return motorSpeed; }, [this] (double set) { motorSpeed = set;});
    builder.AddDoubleProperty("Max Position", [this] { return maxPosition.value(); }, [this] (double set) { maxPosition = units::turn_t{set};});
    ADD_DEFAULT_COMMAND;
    ADD_CURRENT_COMMAND;
}