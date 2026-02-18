#include "subsystems/Launcher.h"

Launcher::Launcher()
{
    launcherMotor1.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration launcherMotor1Config;
    launcherMotor1Config.CurrentLimits.StatorCurrentLimitEnable = true;
    launcherMotor1Config.CurrentLimits.StatorCurrentLimit = 120_A;
    launcherMotor1.GetConfigurator().Apply(launcherMotor1Config);

    launcherMotor2.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration launcherMotor2Config;
    launcherMotor2Config.CurrentLimits.StatorCurrentLimitEnable = true;
    launcherMotor2Config.CurrentLimits.StatorCurrentLimit = 120_A;
    launcherMotor2.GetConfigurator().Apply(launcherMotor2Config);

    launcherMotor3.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration launcherMotor3Config;
    launcherMotor3Config.CurrentLimits.StatorCurrentLimitEnable = true;
    launcherMotor3Config.CurrentLimits.StatorCurrentLimit = 120_A;
    launcherMotor3.GetConfigurator().Apply(launcherMotor3Config);
}


void Launcher::SetLauncherPosition(double position)
{
    actuator1.SetSpeed(position);
    actuator2.SetSpeed(position);
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

frc2::CommandPtr Launcher::RunLauncherCommand(units::turns_per_second_t omega)
{
    return StartEnd(
        [this, omega]
        {
            SetLauncherSpeed(omega);
        },
        [this]
        {
            StopLauncher();
        });
}