#include "subsystems/Hopper.h"

Floor::Floor()
{
    // Basic Configs that both motors will have
    configs::TalonFXConfiguration motorConfigs;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 60_A;
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = 20_A;
    motorConfigs.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
    motorConfigs.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

    motorConfigs = motorConfigs.WithSlot0
    (
        configs::Slot0Configs{}
        .WithKP(HopperConstants::Floor::kP)
        .WithKI(HopperConstants::Floor::kI)
        .WithKD(HopperConstants::Floor::kD)
        .WithKS(HopperConstants::Floor::kS)
        .WithKV(HopperConstants::Floor::kV)
        .WithKA(HopperConstants::Floor::kA)
    );

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
    return RunOnce([this]{ floorMotor.SetControl(floorVelocityRequest.WithVelocity(floorMotorSpeed)); }).WithName("Feed Launcher");
}

frc2::CommandPtr Floor::EjectCommand()
{
    return Run([this] { floorMotor.Set(HopperConstants::Floor::kEjectSpeed); });
}

void Floor::InitSendable(wpi::SendableBuilder &builder)
{
    builder.AddDoubleProperty("Floor Set Speed (rpm)", [this] { return floorMotorSpeed.value(); }, [this] (double set) { floorMotorSpeed = units::revolutions_per_minute_t{set};});
    builder.AddDoubleProperty("Floor Speed (rpm)", [this] { return floorMotor.GetVelocity().GetValue().convert<units::revolutions_per_minute>().value(); }, nullptr);
    builder.AddDoubleProperty("Supply Current", [this] { return floorMotor.GetSupplyCurrent().GetValueAsDouble(); }, nullptr);
    builder.AddDoubleProperty("Stator Current", [this] { return floorMotor.GetStatorCurrent().GetValueAsDouble(); }, nullptr);
    ADD_DEFAULT_COMMAND;
    ADD_CURRENT_COMMAND;
}


Feeder::Feeder()
{
    configs::TalonFXConfiguration motorConfigs;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = 60_A;
    motorConfigs.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
    motorConfigs.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
    motorConfigs = motorConfigs.WithSlot0
    (
        configs::Slot0Configs{}
        .WithKP(HopperConstants::Feeder::kP)
        .WithKI(HopperConstants::Feeder::kI)
        .WithKD(HopperConstants::Feeder::kD)
        .WithKS(HopperConstants::Feeder::kS)
        .WithKV(HopperConstants::Feeder::kV)
        .WithKA(HopperConstants::Feeder::kA)
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
    return Run([this] { feederMotor.Set(HopperConstants::Feeder::kEjectSpeed); });
}

void Feeder::InitSendable(wpi::SendableBuilder &builder)
{
    builder.AddDoubleProperty("Feeder Set Speed (rpm)", [this] { return feederMotorSpeed.value(); }, [this] (double set) { feederMotorSpeed = units::revolutions_per_minute_t{set};});
    builder.AddDoubleProperty("Feeder Speed (rpm)", [this] { return feederMotor.GetVelocity().GetValue().convert<units::revolutions_per_minute>().value(); }, nullptr);
    builder.AddDoubleProperty("Supply Current", [this] { return feederMotor.GetSupplyCurrent().GetValueAsDouble(); }, nullptr);
    builder.AddDoubleProperty("Stator Current", [this] { return feederMotor.GetStatorCurrent().GetValueAsDouble(); }, nullptr);
    ADD_DEFAULT_COMMAND;
    ADD_CURRENT_COMMAND;
}

