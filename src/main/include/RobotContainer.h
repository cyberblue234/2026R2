// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandJoystick.h>

#include "Constants.h"

#include "subsystems/Intake.h"
#include "subsystems/Hopper.h"
#include "subsystems/Launcher.h"
#include "subsystems/Climber.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer
{
public:
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();

private:
    // Replace with CommandPS4Controller or CommandJoystick if needed
    frc2::CommandXboxController joystick{
        OperatorConstants::kDriverControllerPort};

    frc2::CommandJoystick controlBoard{
        OperatorConstants::kControlBoardPort};

    IntakeRoller intakeRoller;
    IntakePivot intakePivot;
    Hopper hopper;
    Launcher launcher;
    Climber climber1{RobotMap::Climber::kClimberMotor1ID, RobotMap::Climber::kClimberLimitSwitch1ID};
    Climber climber2{RobotMap::Climber::kClimberMotor2ID, RobotMap::Climber::kClimberLimitSwitch2ID};

    void ConfigureBindings();
};
