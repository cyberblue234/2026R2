#include "subsystems/Climber.h"

Climber::Climber(int motorID, int limitSwitchID) : climberMotor(motorID), climberLimitSwitch(limitSwitchID)
{
    climberMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration climberMotorConfig;
    climberMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climberMotorConfig.CurrentLimits.StatorCurrentLimit = 120_A;
    climberMotor.GetConfigurator().Apply(climberMotorConfig);

    climberLimitSwitchTrigger.Debounce(60_ms).OnTrue(StopClimberCommand().AndThen(ResetClimberEncoderCommand()));
}

void Climber::ExtendClimber() {
    climberMotor.Set(climberMotorSpeed);
    
}

void Climber::RetractClimber() {
    climberMotor.Set(-climberMotorSpeed);
    
}

void Climber::StopClimber() {
    climberMotor.StopMotor();
}

frc2::CommandPtr Climber::ResetClimberEncoderCommand() {
    return RunOnce([this] 
    {
        climberMotor.SetPosition(0_tr);
        isRegistered = true;
    });
}

frc2::CommandPtr Climber::StopClimberCommand() {
    return RunOnce([this] 
    {
        StopClimber();
    });
}

frc2::CommandPtr Climber::ExtendClimberCommand()
{
    return StartEnd
    (
        [this]
        {
            ExtendClimber();
        },
        [this]
        {
            StopClimber();
        }
    );
}

frc2::CommandPtr Climber::ExtendClimberWithLimitCommand()
{
    return ExtendClimberCommand()
    .OnlyIf
    (
        [this]
        {
            return isRegistered;
        }
    ).Until
    (
        [this] 
        { 
            return GetClimberPosition() > ClimberConstants::kMaxPosition; 
        }
    );
}

frc2::CommandPtr Climber::RetractClimberCommand()
{
    return StartEnd
    (
        [this]
        {
            RetractClimber();
        },
        [this]
        {
            StopClimber();
        }
    );
}