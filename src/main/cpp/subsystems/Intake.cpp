#include "subsystems/Intake.h"

// IntakeRoller

IntakeRoller::IntakeRoller()
{
    rollerMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration rollerMotorConfig;
    rollerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerMotorConfig.CurrentLimits.StatorCurrentLimit = 120_A;
    rollerMotor.GetConfigurator().Apply(rollerMotorConfig);
}

void IntakeRoller::SetRollerMotor(double speed)
{
    rollerMotor.Set(speed);
}

void IntakeRoller::StopRollerMotor()
{
    rollerMotor.StopMotor();
}

frc2::CommandPtr IntakeRoller::StartIntakeCommand()
{
    return Run([this] { SetRollerMotor(IntakeConstants::kIntakeRollerSpeed); })
        .FinallyDo([this] { StopRollerMotor(); })
        .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf);
}

frc2::CommandPtr IntakeRoller::EjectCommand()
{
    return Run([this] { SetRollerMotor(-IntakeConstants::kIntakeRollerSpeed); })
        .FinallyDo([this] { StopRollerMotor(); })
        .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming); 
}

// IntakePivot

IntakePivot::IntakePivot()
{
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

void IntakePivot::SetPosition(units::degree_t angle)
{
    pivotMotor.SetControl(pivotMotorPositionControl.WithPosition(angle));
}

void IntakePivot::SetPositionToGround()
{
    SetPosition(IntakeConstants::kGroundPosition);
}

void IntakePivot::SetPositionToHome()
{
    SetPosition(IntakeConstants::kHomePosition);
}

void IntakePivot::SetPositionToBounce()
{
    SetPosition(IntakeConstants::kBouncePosition);
}

void IntakePivot::StopPivotMotor()
{
    pivotMotor.StopMotor();
}

bool IntakePivot::IsPivotWithinTolerance()
{
    return abs(pivotMotor.GetClosedLoopError().GetValue()) < IntakeConstants::kPivotTolerance.value();
}

frc2::CommandPtr IntakePivot::SetSpeedCommand(double speed)
{
    return Run([this, speed] { pivotMotor.Set(speed); })
        .FinallyDo([this] { StopPivotMotor(); });
}

frc2::CommandPtr IntakePivot::SetPositionToGroundCommand()
{
    return Run([this] { SetPositionToGround(); })
        .FinallyDo([this] { StopPivotMotor(); });
}

frc2::CommandPtr IntakePivot::SetPositionToHomeCommand()
{
    return Run([this] { SetPositionToHome(); })
        .FinallyDo([this] { StopPivotMotor(); });
}

frc2::CommandPtr IntakePivot::SetPositionToBounceCommand()
{
    return Run([this] { SetPositionToBounce(); })
        .FinallyDo([this] { StopPivotMotor(); });
}


frc2::CommandPtr IntakePivot::BounceCommand()
{
    return frc2::cmd::RepeatingSequence
    (
        SetPositionToBounceCommand().Until([this] { return IsPivotWithinTolerance(); }),
        SetPositionToGroundCommand().Until([this] { return IsPivotWithinTolerance(); })
    ).FinallyDo
    (
        [this]
        {
            
            StopPivotMotor();
        }
    );
}