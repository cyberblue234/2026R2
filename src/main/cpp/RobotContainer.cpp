// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureBindings();
    // fuelUpdateCommand.Schedule();

    // if (frc::RobotBase::IsSimulation())
    // {
        frc2::CommandScheduler::GetInstance().Schedule(fuelUpdateCommand);
    // }
}

void RobotContainer::ConfigureBindings()
{
    // Configure your trigger bindings here

    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
        })
    );

    controlBoard.Button(OperatorConstants::kLaunchButton).WhileTrue(frc2::cmd::Sequence(frc2::cmd::RunOnce([this] {simFuelManager.ShootActivated(); }), frc2::cmd::Wait(40_ms)).Repeatedly());

    controlBoard.Button(OperatorConstants::kIntakeSwitch).WhileTrue(intakeRoller.StartIntakeCommand().Repeatedly());
    controlBoard.Button(OperatorConstants::kEjectButton).WhileTrue(intakeRoller.EjectCommand());

    controlBoard.Button(OperatorConstants::kIntakeGroundSwitch).WhileTrue(intakePivot.SetPositionToGroundCommand()); // Mutually exclusive with kIntakeHomeSwitch
    controlBoard.Button(OperatorConstants::kIntakeHomeSwitch).WhileTrue(intakePivot.SetPositionToHomeCommand());

    controlBoard.Button(OperatorConstants::kClimberExtendSwitch).WhileTrue
    (   
        frc2::cmd::Parallel(climber1.ExtendClimberWithLimitCommand(), climber2.ExtendClimberWithLimitCommand())
    );

    controlBoard.Button(OperatorConstants::kClimberRetractSwitch).WhileTrue
    (
        frc2::cmd::Parallel(climber1.RetractClimberCommand(), climber2.RetractClimberCommand())
    );

    // controlBoard.Button(OperatorConstants::kLaunchButton).WhileFalse(simFuelManager.UpdateFuel([this] { return frc::Pose2d{}; }, [this] { return units::radians_per_second_t{}; }, [this] { return 0_deg; }, [this] { return false;} ).IgnoringDisable(true).Repeatedly());
    // controlBoard.Button(OperatorConstants::kIntakeSwitch).OnTrue(simFuelManager.InstantiateFuelCommand());//.OnlyIf(frc::RobotBase::IsSimulation));
}
