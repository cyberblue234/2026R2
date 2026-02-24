#include "subsystems/Launcher.h"

Launcher::Launcher()
{
    configs::Slot0Configs pidConfigs;
    pidConfigs.kP = 2;
    pidConfigs.kI = 0;
    pidConfigs.kD = 0;
    pidConfigs.kS = 0;
    pidConfigs.kV = 2;
    pidConfigs.kA = 0;
    configs::MotionMagicConfigs motionMagicConfigs;
    motionMagicConfigs.MotionMagicCruiseVelocity = 100_rad_per_s;
    motionMagicConfigs.MotionMagicAcceleration = 1000_rad_per_s_sq;

    configs::TalonFXConfiguration launcherMotorConfig;
    launcherMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    launcherMotorConfig.CurrentLimits.StatorCurrentLimit = 120_A;
    launcherMotorConfig.Slot0 = pidConfigs;
    launcherMotorConfig.MotionMagic = motionMagicConfigs;

    launcherMotor1.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    launcherMotor1.GetConfigurator().Apply(launcherMotorConfig);

    launcherMotor2.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    launcherMotor2.GetConfigurator().Apply(launcherMotorConfig);

    launcherMotor3.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    launcherMotor3.GetConfigurator().Apply(launcherMotorConfig);

    deflectorCANcoder.GetConfigurator().Apply(configs::CANcoderConfiguration{});
    configs::CANcoderConfiguration deflectorCANcoderConfig;
    deflectorCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = LauncherConstants::kDiscontinuityPointAngle;
    deflectorCANcoderConfig.MagnetSensor.MagnetOffset = LauncherConstants::kMagnetOffset;
    deflectorCANcoder.GetConfigurator().Apply(deflectorCANcoderConfig);
}


void Launcher::SetLauncherPosition(double position)
{
    actuator1.SetSpeed(position);
    actuator2.SetSpeed(position);
}

void Launcher::SetLauncherAngle(units::degree_t angle)
{   
    deflectorCANcoder.SetPosition(angle);

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

bool Launcher::IsLauncherSpeedWithinTolerance()
{
    return false;
}

frc2::CommandPtr Launcher::ManualSetPosition(double position)
{
    return RunOnce([this, position]
    {
        SetLauncherPosition(position);
    });
}

frc2::CommandPtr Launcher::LaunchCommand(std::function<LauncherState()> setState)
{
    return RunOnce([this, setState]
    {
        currentState = setState();
    }).AndThen([this]
    {
        SetLauncherSpeed(currentState.omega);
        SetLauncherAngle(currentState.pitch);
    }).FinallyDo([this]
    {
        StopLauncher();
    });
}