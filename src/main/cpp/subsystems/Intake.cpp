#include "subsystems/Intake.h"

// IntakeRoller

IntakeRoller::IntakeRoller()
{
    rollerMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration rollerMotorConfig;
    rollerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerMotorConfig.CurrentLimits.StatorCurrentLimit = 120_A;
    rollerMotor.GetConfigurator().Apply(rollerMotorConfig);

    SetDefaultCommand(StopMotorCommand());
}

void IntakeRoller::SetMotor(double speed)
{
    rollerMotor.Set(speed);
}

void IntakeRoller::StopMotor()
{
    rollerMotor.StopMotor();
}

frc2::CommandPtr IntakeRoller::StopMotorCommand()
{
    return RunOnce([this] { StopMotor(); }).WithName("Stop Motor");
}

frc2::CommandPtr IntakeRoller::IntakeCommand()
{
    return Run([this] { SetMotor(motorSpeed); })
        .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf)
        .Repeatedly()
        .WithName("Intake");
}

frc2::CommandPtr IntakeRoller::EjectCommand()
{
    return Run([this] { SetMotor(-motorSpeed); })
        .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
        .WithName("Eject"); 
}


void IntakeRoller::InitSendable(wpi::SendableBuilder &builder)
{
    builder.AddDoubleProperty("Set Speed", [this] { return motorSpeed; }, [this] (double set) { motorSpeed = set;});
    ADD_DEFAULT_COMMAND;
    ADD_CURRENT_COMMAND;
}


// IntakePivot

IntakePivot::IntakePivot()
{
    pivotMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});

    configs::TalonFXConfiguration pivotMotorConfig;
    
    pivotMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotMotorConfig.CurrentLimits.StatorCurrentLimit = 120_A;

    pivotMotorConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    pivotMotorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    pivotMotorConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;
    pivotMotorConfig.Feedback.FeedbackRemoteSensorID = pivotCancoder.GetDeviceID();
    pivotMotorConfig.Feedback.RotorToSensorRatio = IntakeConstants::kMotorToCANcoderRatio;
    pivotMotorConfig.Feedback.SensorToMechanismRatio = IntakeConstants::kCANcoderToPivotRatio;

    // pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants::kGroundPosition;
    // pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants::kHomePosition;

    pivotMotorConfig.Slot0.kP = IntakeConstants::kP;
    pivotMotorConfig.Slot0.kI = IntakeConstants::kI;
    pivotMotorConfig.Slot0.kD = IntakeConstants::kD;
    pivotMotorConfig.Slot0.GravityType = signals::GravityTypeValue::Arm_Cosine;
    pivotMotorConfig.Slot0.GravityArmPositionOffset = IntakeConstants::kGravityArmPositionOffset;
    pivotMotorConfig.Slot0.kS = IntakeConstants::kS;
    pivotMotorConfig.Slot0.kG = IntakeConstants::kG;
    pivotMotorConfig.Slot0.kV = IntakeConstants::kV;
    pivotMotorConfig.Slot0.kA = IntakeConstants::kA;

    pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants::kMaxVelocity;
    pivotMotorConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants::kMaxAcceleration;
    pivotMotorConfig.MotionMagic.MotionMagicJerk = IntakeConstants::kMaxJerk;

    pivotMotor.GetConfigurator().Apply(pivotMotorConfig);

    pivotCancoder.GetConfigurator().Apply(configs::CANcoderConfiguration{});
    configs::CANcoderConfiguration pivotCancoderConfig;
    pivotCancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = IntakeConstants::kDiscontinuityPointAngle;
    pivotCancoderConfig.MagnetSensor.MagnetOffset = IntakeConstants::kMagnetOffset;
    pivotCancoderConfig.MagnetSensor.SensorDirection = signals::SensorDirectionValue::Clockwise_Positive;
    pivotCancoder.GetConfigurator().Apply(pivotCancoderConfig);

    SetDefaultCommand(StopMotorCommand());
}

void IntakePivot::SetPosition(units::degree_t angle)
{
    positionController.SetSetpoint(angle.value());
    pivotMotor.SetControl(positionVoltageOut.WithOutput(units::volt_t{positionController.Calculate(GetAngle().value())}));
}

void IntakePivot::SetPositionToGround()
{
    SetPosition(groundPosition);
}

void IntakePivot::SetPositionToHome()
{
    SetPosition(homePosition);
}

void IntakePivot::SetPositionToBounce()
{
    SetPosition(bouncePosition);
}

void IntakePivot::StopMotor()
{
    pivotMotor.StopMotor();
}

bool IntakePivot::IsWithinTolerance()
{
    return units::math::abs(units::degree_t{positionController.GetSetpoint()} - GetAngle()) < tolerance;
}

frc2::CommandPtr IntakePivot::StopMotorCommand()
{
    return RunOnce([this] { StopMotor(); }).WithName("Stop Motor");
}

frc2::CommandPtr IntakePivot::SetMotorToBrakeCommand()
{
    return Run([this] { pivotMotor.SetControl(controls::StaticBrake{}); }).WithName("Set Motor to Coast");
}

frc2::CommandPtr IntakePivot::SetSpeedCommand(double speed)
{
    return Run([this, speed] 
        { 
            pivotMotor.SetControl(positionVoltageOut.WithOutput(12_V * speed)
            .WithIgnoreSoftwareLimits(true)); 
        }).FinallyDo([this] { positionVoltageOut.Output = 0_V; }).WithName("Set Speed");
}

frc2::CommandPtr IntakePivot::SetPositionCommand(std::function<units::degree_t()> angle)
{
    return Run([this, angle] { SetPosition(angle()); }).BeforeStarting([this, angle] { positionController.SetSetpoint(angle().value()); }).FinallyDo([this] { positionVoltageOut.Output = 0_V; }).WithName("Set Position");
}

frc2::CommandPtr IntakePivot::SetPositionToGroundCommand()
{
    return SetPositionCommand([this] { return groundPosition; }).WithName("Set Position to Ground");
}

frc2::CommandPtr IntakePivot::SetPositionToHomeCommand()
{
    return SetPositionCommand([this] { return homePosition; }).AndThen(SetMotorToBrakeCommand()).WithName("Set Position to Home");
}

frc2::CommandPtr IntakePivot::SetPositionToBounceCommand()
{
    return SetPositionCommand([this] { return bouncePosition; }).WithName("Set Position to Bounce");
}


frc2::CommandPtr IntakePivot::BounceCommand()
{
    return frc2::cmd::RepeatingSequence
    (
        SetPositionToBounceCommand().Until([this] { return IsWithinTolerance(); }),
        SetPositionCommand([this] { return 140_deg; }).Until([this] { return IsWithinTolerance(); })
    ).WithName("Bounce");
}

void IntakePivot::SimulationPeriodic()
{
    sim::TalonFXSimState& sim = pivotMotor.GetSimState();
    sim.SetMotorType(sim::TalonFXSimState::MotorType::KrakenX60);
    sim::CANcoderSimState& cancoderSim = pivotCancoder.GetSimState();
    sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    intakeSim.SetInputVoltage(positionVoltageOut.Output);
    intakeSim.Update(20_ms);
    cancoderSim.SetRawPosition(-intakeSim.GetAngle() * IntakeConstants::kCANcoderToPivotRatio);
    sim.SetRawRotorPosition(-intakeSim.GetAngle() * IntakeConstants::kMotorToPivotRatio);
    sim.SetRotorVelocity(-intakeSim.GetVelocity() * IntakeConstants::kMotorToPivotRatio);
}

void IntakePivot::InitSendable(wpi::SendableBuilder &builder)
{
    builder.AddDoubleProperty("Ground Position", [this] { return groundPosition.value(); }, [this] (double set) { groundPosition = units::degree_t{set};});
    builder.AddDoubleProperty("Home Position", [this] { return homePosition.value(); }, [this] (double set) { homePosition = units::degree_t{set};});
    builder.AddDoubleProperty("Bounce Position", [this] { return bouncePosition.value(); }, [this] (double set) { bouncePosition = units::degree_t{set};});
    builder.AddDoubleProperty("Tolerance", [this] { return tolerance.value(); }, [this] (double set) { tolerance = units::degree_t{set};});
    builder.AddBooleanProperty("Is Within Tolerance", [this] { return IsWithinTolerance(); }, nullptr);
    builder.AddDoubleProperty("Goal", [this] { return positionController.GetSetpoint(); }, nullptr);
    builder.AddDoubleProperty("Setpoint", [this] { return units::turn_t(pivotMotor.GetClosedLoopReference().GetValue()).convert<units::degree>().value(); }, nullptr);
    builder.AddDoubleProperty("Current Position", [this] { return GetAngle().value(); }, nullptr);
    ADD_DEFAULT_COMMAND;
    ADD_CURRENT_COMMAND;
}