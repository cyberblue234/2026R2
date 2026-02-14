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

	controlBoard.Button(OperatorConstants::kClimberExtendSwitch).WhileTrue
	(
		climber.ExtendClimberCommand()
		.Until
		(
			[this] 
			{ 
				return climber.GetClimberPosition() > ClimberConstants::kMaxPosition; 
			}
		)
	);
	controlBoard.Button(OperatorConstants::kClimberRetractSwitch).WhileTrue(climber.RetractClimberCommand());
}
