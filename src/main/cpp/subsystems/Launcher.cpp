#include "subsystems/Launcher.h"

Launcher::Launcher()
{
    configs::Slot0Configs pidConfigs;
    pidConfigs.kP = LauncherConstants::kP;
    pidConfigs.kI = LauncherConstants::kI;
    pidConfigs.kD = LauncherConstants::kD;
    pidConfigs.kS = LauncherConstants::kS;
    pidConfigs.kV = LauncherConstants::kV;
    pidConfigs.kA = LauncherConstants::kA;
    configs::MotionMagicConfigs motionMagicConfigs;
    motionMagicConfigs.MotionMagicCruiseVelocity = LauncherConstants::kCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = LauncherConstants::kAcceleration;

    configs::TalonFXConfiguration launcherMotorConfig;
    launcherMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    launcherMotorConfig.CurrentLimits.StatorCurrentLimit = 120_A;
    launcherMotorConfig.Slot0 = pidConfigs;
    launcherMotorConfig.MotionMagic = motionMagicConfigs;
    launcherMotorConfig.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;

    launcherMotor1.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    launcherMotor1.GetConfigurator().Apply(launcherMotorConfig);

    launcherMotor2.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    launcherMotor2.GetConfigurator().Apply(launcherMotorConfig);

    launcherMotor3.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    launcherMotorConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    launcherMotor3.GetConfigurator().Apply(launcherMotorConfig);

    deflectorCANcoder.GetConfigurator().Apply(configs::CANcoderConfiguration{});
    configs::CANcoderConfiguration deflectorCANcoderConfig;
    deflectorCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = LauncherConstants::kDiscontinuityPointAngle;
    deflectorCANcoderConfig.MagnetSensor.MagnetOffset = LauncherConstants::kMagnetOffset;
    deflectorCANcoderConfig.MagnetSensor.SensorDirection = signals::InvertedValue::CounterClockwise_Positive;
    deflectorCANcoder.GetConfigurator().Apply(deflectorCANcoderConfig);

    SetDefaultCommand(StopMotorsCommand());
}

void Launcher::LowerDeflector()
{
    deflectorRelay.Set(frc::Relay::Value::kReverse);
}

void Launcher::RaiseDeflector()
{
    deflectorRelay.Set(frc::Relay::Value::kForward);
}

void Launcher::StopDeflector()
{
    deflectorRelay.Set(frc::Relay::Value::kOff);
}

void Launcher::SetLauncherAngle(units::degree_t angle, units::degree_t tolerance)
{   
    units::degree_t error = GetLauncherAngle() - angle;
    if (units::math::abs(error) > tolerance)
    {
        if (error >= 0_deg) RaiseDeflector();
        else LowerDeflector();
    }
    else
    {
        StopDeflector();
    }
    if (frc::RobotBase::IsSimulation())
    {
        ctre::phoenix6::sim::CANcoderSimState& sim = deflectorCANcoder.GetSimState();
        sim.AddPosition(angle - deflectorCANcoder.GetAbsolutePosition().GetValue());
    }
}

void Launcher::SetLauncherSpeed(units::turns_per_second_t omega)
{
    launcherMotorVelocityControl.Velocity = omega;
    launcherMotor1.SetControl(launcherMotorVelocityControl);
    launcherMotor2.SetControl(launcherMotorVelocityControl);
    launcherMotor3.SetControl(launcherMotorVelocityControl);
}

void Launcher::StopLauncher()
{
    launcherMotor1.StopMotor();
    launcherMotor2.StopMotor();
    launcherMotor3.StopMotor();
}

bool Launcher::IsLauncherSpeedWithinTolerance(units::radians_per_second_t tolerance)
{
    return units::math::abs(GetLowestLauncherOmega() - currentState.omega) < tolerance ;
}

void Launcher::Eject()
{
    launcherMotor1.Set(-1);
    launcherMotor2.Set(-1);
    launcherMotor3.Set(-1);
}

frc2::CommandPtr Launcher::ManualRaiseDeflectorCommand()
{
    return Run([this]
    {
        RaiseDeflector();
    });
}
frc2::CommandPtr Launcher::ManualLowerDeflectorCommand()
{
    return Run([this]
    {
        LowerDeflector();
    });
}

frc2::CommandPtr Launcher::StopMotorsCommand()
{
    return RunOnce([this]
    {
        StopLauncher();
    }).WithName("Stop Motors");
}

frc2::CommandPtr Launcher::LaunchCommand(std::function<LauncherState()> setState)
{
    return Run([this, setState]
    {
        currentState = setState();
        SetLauncherSpeed(currentState.omega);
        SetLauncherAngle(currentState.pitch, currentState.tolerance);
    }).WithName("Launch");
}

frc2::CommandPtr Launcher::EjectCommand()
{
    return Run([this] { Eject(); });
}

void Launcher::SimulationPeriodic()
{
    ctre::phoenix6::sim::TalonFXSimState& sim = launcherMotor1.GetSimState();
    sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    launcherSim.SetInputVoltage(sim.GetMotorVoltage());
    launcherSim.Update(20_ms);
    sim.SetRotorVelocity(launcherSim.GetAngularVelocity());   
}

void Launcher::InitSendable(wpi::SendableBuilder& builder)
{
    builder.AddDoubleArrayProperty("Launcher Speed (rpm)", [this] { return GetPublishableLauncherRPMs(); }, nullptr);
    builder.AddDoubleProperty("Launcher Set Speed (rpm)", [this] { return currentState.omega.convert<units::revolutions_per_minute>().value(); }, nullptr);
    builder.AddDoubleProperty("Launcher Set Angle (degrees)", [this] { return currentState.pitch.value(); }, nullptr);
    builder.AddBooleanProperty("Launcher at Speed", [this] { return IsLauncherSpeedWithinTolerance(); }, nullptr);
    builder.AddDoubleProperty("Launcher Angle (deg)", [this] { return GetLauncherAngle().value(); }, nullptr);
    builder.AddDoubleProperty("Actuator 1 Position", [this] { return actuator1.GetSpeed(); }, nullptr);
    builder.AddDoubleProperty("Actuator 2 Position", [this] { return actuator2.GetSpeed(); }, nullptr);
    builder.AddDoubleProperty("Loss Factor", [this] { return loss; }, [this](double newLoss) { loss = newLoss; });
    ADD_DEFAULT_COMMAND;
    ADD_CURRENT_COMMAND;
}