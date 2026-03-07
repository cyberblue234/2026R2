#include "subsystems/Launcher.h"

Launcher::Launcher()
{
    configs::Slot0Configs pidConfigs;
    pidConfigs.kP = 0.5;
    pidConfigs.kI = 0;
    pidConfigs.kD = 0;
    pidConfigs.kS = 0.25;
    pidConfigs.kV = 0.11;
    pidConfigs.kA = 0.1;
    configs::MotionMagicConfigs motionMagicConfigs;
    motionMagicConfigs.MotionMagicCruiseVelocity = 100_rad_per_s;
    motionMagicConfigs.MotionMagicAcceleration = 1000_rad_per_s_sq;

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
    deflectorCANcoder.GetConfigurator().Apply(deflectorCANcoderConfig);

    SetDefaultCommand(StopMotorsCommand());
}


void Launcher::SetLauncherPosition(double position)
{
    actuator1.SetSpeed(position);
    actuator2.SetSpeed(position);
}

void Launcher::SetLauncherAngle(units::degree_t angle)
{   
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
    if (frc::RobotBase::IsSimulation())
    {
        ctre::phoenix6::sim::TalonFXSimState& sim = launcherMotor1.GetSimState();
        sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
        launcherSim.SetInputVoltage(sim.GetMotorVoltage());
        launcherSim.Update(20_ms);
        sim.SetRotorVelocity(launcherSim.GetAngularVelocity());
    }
}

void Launcher::StopLauncher()
{
    launcherMotor1.StopMotor();
    launcherMotor2.StopMotor();
    launcherMotor3.StopMotor();
}

bool Launcher::IsLauncherSpeedWithinTolerance(units::radians_per_second_t tolerance)
{
    return units::math::abs(GetLauncherOmega() - currentState.omega) < tolerance ;
}

frc2::CommandPtr Launcher::ManualSetPosition(double position)
{
    return RunOnce([this, position]
    {
        SetLauncherPosition(position);
    }).WithName("Manual Set Position");
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
        SetLauncherAngle(currentState.pitch);
    }).WithName("Launch");
}

void Launcher::InitSendable(wpi::SendableBuilder& builder)
{
    builder.AddDoubleProperty("Launcher Speed (rpm)", [this] { return GetLauncherOmega().convert<units::revolutions_per_minute>().value(); }, nullptr);
    builder.AddDoubleProperty("Launcher Set Speed (rpm)", [this] { return currentState.omega.convert<units::revolutions_per_minute>().value(); }, nullptr);
    builder.AddDoubleProperty("Launcher Set Angle (degrees)", [this] { return currentState.pitch.value(); }, nullptr);
    builder.AddBooleanProperty("Launcher at Speed", [this] { return IsLauncherSpeedWithinTolerance(); }, nullptr);
    builder.AddDoubleProperty("Launcher Angle (deg)", [this] { return GetLauncherAngle().value(); }, nullptr);
    ADD_DEFAULT_COMMAND;
    ADD_CURRENT_COMMAND;
}