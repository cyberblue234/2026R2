// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

Robot::Robot()  
{
    ctre::phoenix6::SignalLogger::EnableAutoLogging(false);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
    telemetry.UpdateTelemetry();
    container.turretVision.Periodic();
    container.backLeftVision.Periodic();
    container.backRightVision.Periodic();

    frc::SmartDashboard::PutBoolean("Generic/Won Autonomous", wonAuto);

    if (!configuredWonAuto && !frc::DriverStation::IsAutonomous())
    {   
        std::string gameData;
        gameData = frc::DriverStation::GetGameSpecificMessage();
        if (gameData.length() > 0)
        {
            configuredWonAuto = true;
            switch (gameData[0])
            {
                case 'B' :
                    wonAuto = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;
                    break;
                case 'R' :
                    wonAuto = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;
                    break;
                default :
                    configuredWonAuto = false;
                    break;
            }
        }
    }

    units::second_t time = frc::DriverStation::GetMatchTime();
    frc::SmartDashboard::PutNumber("Generic/Match Time", time.value());
    if (frc::DriverStation::IsTeleop())
    {
        // Endgame period
        if (time <= GameConstants::kEndgamePeriod)
        {
            frc::SmartDashboard::PutNumber("Generic/Shift Time", time.value());
            frc::SmartDashboard::PutBoolean("Generic/Hub Active", true);
        }
        // Fourth shift
        else if (time <= GameConstants::kEndgamePeriod + GameConstants::kShiftTimes)
        {
            frc::SmartDashboard::PutNumber("Generic/Shift Time", (time - GameConstants::kEndgamePeriod).value());
            frc::SmartDashboard::PutBoolean("Generic/Hub Active", wonAuto);
        }
        // Third shift
        else if (time <= GameConstants::kEndgamePeriod + 2 * GameConstants::kShiftTimes)
        {
            frc::SmartDashboard::PutNumber("Generic/Shift Time", (time - GameConstants::kEndgamePeriod - GameConstants::kShiftTimes).value());
            frc::SmartDashboard::PutBoolean("Generic/Hub Active", !wonAuto);
        }
        // Second shift
        else if (time <= GameConstants::kEndgamePeriod + 3 * GameConstants::kShiftTimes)
        {
            frc::SmartDashboard::PutNumber("Generic/Shift Time", (time - GameConstants::kEndgamePeriod - 2 * GameConstants::kShiftTimes).value());
            frc::SmartDashboard::PutBoolean("Generic/Hub Active", wonAuto);
        }
        // First shift
        else if (time <= GameConstants::kEndgamePeriod + 4 * GameConstants::kShiftTimes)
        {
            frc::SmartDashboard::PutNumber("Generic/Shift Time", (time - GameConstants::kEndgamePeriod - 3 * GameConstants::kShiftTimes).value());
            frc::SmartDashboard::PutBoolean("Generic/Hub Active", !wonAuto);
        }
        // Transition period
        else if (time <= GameConstants::kEndgamePeriod + 4 * GameConstants::kShiftTimes + GameConstants::kTransitionShiftTime)
        {
            frc::SmartDashboard::PutNumber("Generic/Shift Time", (time - GameConstants::kEndgamePeriod - 4 * GameConstants::kShiftTimes).value());
            frc::SmartDashboard::PutBoolean("Generic/Hub Active", true);
        }
    }
    else
    {
        frc::SmartDashboard::PutNumber("Generic/Shift Time", -1);
    }
    
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() 
{
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit()
{
    autonomousCommand = container.GetAutonomousCommand();
    if (autonomousCommand)
    {
        frc2::CommandScheduler::GetInstance().Schedule(autonomousCommand.value());
    }
    frc::SmartDashboard::PutBoolean("Generic/Hub Active", true);

    frc::DataLogManager::Start();
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand)
    {
        autonomousCommand->Cancel();
    }
    frc2::CommandScheduler::GetInstance().Schedule(container.IntakePivotDefaultCommand());
    frc::SmartDashboard::PutBoolean("Generic/Hub Active", false);

    frc::DataLogManager::Start();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() 
{
    frc::Pose2d robotPose = container.drivetrain.GetState().Pose;
    container.turretVision.SimPeriodic(robotPose);
    container.backLeftVision.SimPeriodic(robotPose);
    container.backRightVision.SimPeriodic(robotPose);
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
