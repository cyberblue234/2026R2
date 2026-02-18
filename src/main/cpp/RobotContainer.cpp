// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Configure your trigger bindings here

    controlBoard.Button(OperatorConstants::kAutoIntakeSwitch).WhileTrue(intake.IntakeFuelCommand());

    controlBoard.Button(OperatorConstants::kIntakeSwitch).WhileTrue(intake.ManualIntakeCommand());

    controlBoard.Button(OperatorConstants::kIntakeGroundSwitch).WhileTrue(intake.SetPositionToGroundCommand());

    controlBoard.Button(OperatorConstants::kIntakeHomeSwitch).WhileTrue(intake.SetPositionToHomeCommand());

    controlBoard.Button(OperatorConstants::kClimberExtendSwitch).WhileTrue(frc2::cmd::Parallel(climber1.ExtendClimberWithLimitCommand(), climber2.ExtendClimberWithLimitCommand()));

    controlBoard.Button(OperatorConstants::kClimberRetractSwitch).WhileTrue(frc2::cmd::Parallel(climber1.RetractClimberCommand(), climber2.RetractClimberCommand()));
}
