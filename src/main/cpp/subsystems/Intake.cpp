#include "subsystems/Intake.h"

// IntakeRoller

IntakeRoller::IntakeRoller()
{
    rollerMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration rollerMotorConfig;
    rollerMotorConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    rollerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerMotorConfig.CurrentLimits.StatorCurrentLimit = 80_A;
    rollerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
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
    builder.AddDoubleProperty("Motor Speed (rpm)", [this] { return rollerMotor.GetVelocity().GetValue().convert<units::revolutions_per_minute>().value(); }, nullptr);
    builder.AddDoubleProperty("Supply Current", [this] { return rollerMotor.GetSupplyCurrent().GetValueAsDouble(); }, nullptr);
    builder.AddDoubleProperty("Stator Current", [this] { return rollerMotor.GetStatorCurrent().GetValueAsDouble(); }, nullptr);
    ADD_DEFAULT_COMMAND;
    ADD_CURRENT_COMMAND;
}


// IntakePivot

IntakePivot::IntakePivot()
{
    pivotMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});

    configs::TalonFXConfiguration pivotMotorConfig;
    
    pivotMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotMotorConfig.CurrentLimits.StatorCurrentLimit = 25_A;
    pivotMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    pivotMotorConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    pivotMotorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    pivotMotor.GetConfigurator().Apply(pivotMotorConfig);

    SetDefaultCommand(StopMotorCommand());
}

void IntakePivot::StopMotor()
{
    pivotMotor.StopMotor();
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

frc2::CommandPtr IntakePivot::SetPositionToGroundCommand()
{
    return SetSpeedCommand(IntakeConstants::kManualSpeed);
}

frc2::CommandPtr IntakePivot::SetPositionToHomeCommand()
{
    return SetSpeedCommand(-IntakeConstants::kManualSpeed).WithTimeout(1_s);
}

frc2::CommandPtr IntakePivot::BounceCommand()
{
    return frc2::cmd::RepeatingSequence
    (
        SetSpeedCommand(-IntakeConstants::kManualSpeed).WithTimeout(0.25_s),
        SetSpeedCommand(0.1).WithTimeout(0.4_s)
    ).WithName("Bounce");
}

void IntakePivot::SimulationPeriodic()
{
    sim::TalonFXSimState& sim = pivotMotor.GetSimState();
    sim.SetMotorType(sim::TalonFXSimState::MotorType::KrakenX60);
    sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    intakeSim.SetInputVoltage(positionVoltageOut.Output);
    intakeSim.Update(20_ms);
    sim.SetRawRotorPosition(-intakeSim.GetAngle() * IntakeConstants::kMotorToPivotRatio);
    sim.SetRotorVelocity(-intakeSim.GetVelocity() * IntakeConstants::kMotorToPivotRatio);
}

void IntakePivot::InitSendable(wpi::SendableBuilder &builder)
{
    builder.AddDoubleProperty("Supply Current", [this] { return pivotMotor.GetSupplyCurrent().GetValueAsDouble(); }, nullptr);
    builder.AddDoubleProperty("Stator Current", [this] { return pivotMotor.GetStatorCurrent().GetValueAsDouble(); }, nullptr);
    ADD_DEFAULT_COMMAND;
    ADD_CURRENT_COMMAND;
}