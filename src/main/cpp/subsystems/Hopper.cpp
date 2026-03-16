#include "subsystems/Hopper.h"

Floor::Floor()
{
    // Basic Configs that both motors will have
    configs::TalonFXConfiguration motorConfigs;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 120_A;
    motorConfigs.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    motorConfigs.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

    floorMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    floorMotor.GetConfigurator().Apply(motorConfigs);

    SetDefaultCommand(StopCommand());
}

frc2::CommandPtr Floor::StopCommand()
{
    return RunOnce([this] { floorMotor.StopMotor(); }).WithName("Stop Motors");
}

frc2::CommandPtr Floor::FeedCommand()
{
    return RunOnce([this]{ floorMotor.Set(floorMotorSpeed); }).WithName("Feed Launcher");
}

frc2::CommandPtr Floor::EjectCommand()
{
    return Run([this] { floorMotor.Set(-floorMotorSpeed); });
}

void Floor::InitSendable(wpi::SendableBuilder &builder)
{
    builder.AddDoubleProperty("Floor Set Speed", [this] { return floorMotorSpeed; }, [this] (double set) { floorMotorSpeed = set;});
    ADD_DEFAULT_COMMAND;
    ADD_CURRENT_COMMAND;
}


Feeder::Feeder()
{
    configs::TalonFXConfiguration motorConfigs;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 120_A;
    motorConfigs.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
    motorConfigs.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
    motorConfigs = motorConfigs.WithSlot0
    (
        configs::Slot0Configs{}
        .WithKP(HopperConstants::kP)
        .WithKI(HopperConstants::kI)
        .WithKD(HopperConstants::kD)
        .WithKS(HopperConstants::kS)
        .WithKV(HopperConstants::kV)
        .WithKA(HopperConstants::kA)
    );

    feederMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    feederMotor.GetConfigurator().Apply(motorConfigs);

    SetDefaultCommand(StopCommand());
}

frc2::CommandPtr Feeder::StopCommand()
{
    return RunOnce([this] { feederMotor.StopMotor(); }).WithName("Stop Motors");
}

frc2::CommandPtr Feeder::FeedCommand()
{
    return RunOnce([this]{ feederMotor.SetControl(feederVelocityRequest.WithVelocity(feederMotorSpeed)); }).WithName("Feed Launcher");
}

frc2::CommandPtr Feeder::EjectCommand()
{
    return Run([this] { feederMotor.Set(HopperConstants::kFeederEjectSpeed); });
}

void Feeder::InitSendable(wpi::SendableBuilder &builder)
{
    builder.AddDoubleProperty("Feeder Set Speed (rpm)", [this] { return feederMotorSpeed.value(); }, [this] (double set) { feederMotorSpeed = units::revolutions_per_minute_t{set};});
    builder.AddDoubleProperty("Feeder Speed (rpm)", [this] { return feederMotor.GetVelocity().GetValue().convert<units::revolutions_per_minute>().value(); }, nullptr);
    ADD_DEFAULT_COMMAND;
    ADD_CURRENT_COMMAND;
}

