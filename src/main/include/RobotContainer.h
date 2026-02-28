// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc/Joystick.h>

#include <frc/filter/SlewRateLimiter.h>

#include "Constants.h"

#include "generated/FieldCentricFacingAngleProfiled.h"

#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/Intake.h"
#include "subsystems/Hopper.h"
#include "subsystems/Launcher.h"
#include "subsystems/Climber.h"

#include "sim/Fuel.hpp"

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
private:
    frc2::CommandPtr GetAutonomousCommand();

private:
    // Replace with CommandPS4Controller or CommandJoystick if needed
    frc2::CommandXboxController joystick{
        OperatorConstants::kDriverControllerPort};

    frc2::CommandJoystick controlBoard{
        OperatorConstants::kControlBoardPort};
    frc::Joystick controlBoardRegular{
        OperatorConstants::kControlBoardPort
    };

    units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts;
    units::radians_per_second_t MaxAngularRate = 1_tps; 

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.2).WithRotationalDeadband(MaxAngularRate * 0.2) // Add a 20% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);

    double alignmentKP = 8;
    double alignmentKI = 100;
    double alignmentKD = 0;

    swerve::requests::FieldCentricFacingAngleProfiled alignToHub = swerve::requests::FieldCentricFacingAngleProfiled{}
        .WithCenterOfRotation({-LauncherConstants::kTurretOffset.X(), LauncherConstants::kTurretOffset.Y()})
        .WithDeadband(0.2_mps)
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage)
        .WithSteerRequestType(swerve::SteerRequestType::Position)
        .WithHeadingPID(alignmentKP, alignmentKI, alignmentKD)
        .WithTolerance(8_deg);
    
    frc::SlewRateLimiter<units::meters_per_second> DriveXAccelerationLimiter{3_mps_sq};
    frc::SlewRateLimiter<units::meters_per_second> DriveYAccelerationLimiter{3_mps_sq};

    units::degree_t targetYaw;
    units::degree_t pitch;
    units::radians_per_second_t omega;

    units::meter_t kZOffset = 1_m;

public:
    CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};
    IntakeRoller intakeRoller;
    IntakePivot intakePivot;
    Hopper hopper;
    Launcher launcher;
    Climber climber1{RobotMap::Climber::kClimberMotor1ID, RobotMap::Climber::kClimberLimitSwitch1ID};
    Climber climber2{RobotMap::Climber::kClimberMotor2ID, RobotMap::Climber::kClimberLimitSwitch2ID};

    FuelManager simFuelManager;
    frc2::CommandPtr fuelUpdateCommand = simFuelManager.UpdateFuel
    (
        [this] { return frc::Pose3d{drivetrain.GetState().Pose}; }, 
        [this] { return frc::ChassisSpeeds{drivetrain.GetVelocityX(), drivetrain.GetVelocityY(), drivetrain.GetVelocityYaw()}; }, 
        launcher.GetLauncherOmegaSupplier(), 
        launcher.GetLauncherAngleAsSupplier()
    );

    void ConfigureBindings();
};
