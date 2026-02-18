#include "subsystems/Intake.h"

Intake::Intake()
{
    rollerMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration rollerMotorConfig;
    rollerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerMotorConfig.CurrentLimits.StatorCurrentLimit = 120_A;
    rollerMotor.GetConfigurator().Apply(rollerMotorConfig);

    pivotMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration pivotMotorConfig;
    pivotMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotMotorConfig.CurrentLimits.StatorCurrentLimit = 120_A;

    pivotMotorConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;
    pivotMotorConfig.Feedback.FeedbackRemoteSensorID = pivotCancoder.GetDeviceID();
    pivotMotorConfig.Feedback.RotorToSensorRatio = IntakeConstants::kPivotToCANcoderRatio;
    pivotMotorConfig.Feedback.SensorToMechanismRatio = 1;

    pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants::kGroundPosition;
    pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants::kHomePosition;

    pivotMotorConfig.Slot0.kP = 2;
    pivotMotorConfig.Slot0.kI = 0.0;
    pivotMotorConfig.Slot0.kD = 0.1;

    pivotMotor.GetConfigurator().Apply(pivotMotorConfig);

    pivotCancoder.GetConfigurator().Apply(configs::CANcoderConfiguration{});
    configs::CANcoderConfiguration pivotCancoderConfig;
    pivotCancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = IntakeConstants::kDiscontinuityPointAngle;
    pivotCancoderConfig.MagnetSensor.MagnetOffset = IntakeConstants::kMagnetOffset;
    pivotCancoder.GetConfigurator().Apply(pivotCancoderConfig);
}

void Intake::SetRollerMotor(double speed)
{
    rollerMotor.Set(speed);
}

void Intake::StopRollerMotor()
{
    rollerMotor.StopMotor();
}

void Intake::SetPosition(units::degree_t angle)
{
    pivotMotor.SetControl(pivotMotorPositionControl.WithPosition(angle));
}

void Intake::SetPositionToGround()
{
    SetPosition(IntakeConstants::kGroundPosition);
}

void Intake::SetPositionToHome()
{
    SetPosition(IntakeConstants::kHomePosition);
}

void Intake::SetPositionToBounce()
{
    SetPosition(IntakeConstants::kBouncePosition);
}

void Intake::StopPivotMotor()
{
    pivotMotor.StopMotor();
}

bool Intake::IsPivotWithinTolerance()
{
    return abs(pivotMotor.GetClosedLoopError().GetValue()) < IntakeConstants::kPivotTolerance.value();
}

frc2::CommandPtr Intake::ManualIntakeCommand()
{
    return Run([this]
               { SetRollerMotor(IntakeConstants::kIntakeRollerSpeed); })
        .FinallyDo([this]
                   { StopRollerMotor(); });
}

frc2::CommandPtr Intake::SetPositionToGroundCommand()
{
    return Run([this]
               { SetPositionToGround(); })
        .FinallyDo([this]
                   { StopPivotMotor(); });
}

frc2::CommandPtr Intake::SetPositionToHomeCommand()
{
    return Run([this]
               { SetPositionToHome(); })
        .FinallyDo([this]
                   { StopPivotMotor(); });
}

frc2::CommandPtr Intake::IntakeFuelCommand()
{
    return Run([this]
               {
        SetPositionToGround();
        if (IsPivotWithinTolerance())
        {
            SetRollerMotor(IntakeConstants::kIntakeRollerSpeed);
        }
        else 
        {
            pivotMotor.Feed();
        } })
        .FinallyDo([this]
                   {
        StopPivotMotor();
        StopRollerMotor(); });
}

frc2::CommandPtr Intake::BounceCommand()
{
    return StartRun(
               [this]
               {
                   setToGround = true;
               },
               [this]
               {
                   if (setToGround)
                   {
                       SetPositionToGround();
                       if (IsPivotWithinTolerance())
                       {
                           setToGround = false;
                       }
                   }
                   else
                   {
                       SetPositionToBounce();
                       if (IsPivotWithinTolerance())
                       {
                           setToGround = true;
                       }
                   }
               })
        .FinallyDo([this]
                   { StopPivotMotor(); });
}