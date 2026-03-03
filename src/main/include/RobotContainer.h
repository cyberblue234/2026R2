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

namespace RobotContainerConstants
{
    namespace DriveConstants
    {
        inline constexpr units::meters_per_second_t kMaxSpeed = TunerConstants::kSpeedAt12Volts;
        inline constexpr units::radians_per_second_t kMaxAngularRate = 1_tps;

        inline constexpr units::meters_per_second_squared_t kAccelerationLimit = 20_mps_sq;
        inline constexpr units::degrees_per_second_squared_t kAngularAccelerationLimit = 4_tr_per_s_sq;
        inline constexpr double kDeadband = 0.1;
    }

    namespace AlignmentConstants
    {
        inline constexpr double kP = 12;
        inline constexpr double kI = 100;
        inline constexpr double kD = 0;

        inline constexpr units::meters_per_second_t kMaxSpeed = 2.5_mps;
        inline constexpr units::meters_per_second_squared_t kAccelerationLimit = 3_mps_sq;
        inline constexpr double kDeadband = 0.1;

        inline constexpr units::meter_t kToleranceRadius = 16_in;
        inline constexpr units::meter_t kZOffset = 1_m;
    }

    namespace IntakePivotConstants
    {
        inline constexpr double kManualSpeed = 0.1;
    }
}

using namespace RobotContainerConstants;

class RobotContainer
{
public:
    RobotContainer();
private:
    frc2::CommandPtr GetAutonomousCommand();

    bool IsAlignmentWithinTolerances()
    {
        return launcher.IsLauncherSpeedWithinTolerance(omegaTolerance) && units::math::abs(frc::Rotation2d(targetYaw).Degrees() - drivetrain.GetState().Pose.Rotation().Degrees()) < yawTolerance;
    }

    frc2::CommandXboxController joystick{
        OperatorConstants::kDriverControllerPort};

    frc2::CommandJoystick controlBoard{
        OperatorConstants::kControlBoardPort};
    frc::Joystick controlBoardRegular{
        OperatorConstants::kControlBoardPort
    };

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(DriveConstants::kMaxSpeed * DriveConstants::kDeadband)
        .WithRotationalDeadband(DriveConstants::kMaxAngularRate * DriveConstants::kDeadband)
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);
    frc::SlewRateLimiter<units::meters_per_second> driveXLimiter{DriveConstants::kAccelerationLimit};
    frc::SlewRateLimiter<units::meters_per_second> driveYLimiter{DriveConstants::kAccelerationLimit};
    frc::SlewRateLimiter<units::degrees_per_second> driveYawLimiter{DriveConstants::kAngularAccelerationLimit};

    swerve::requests::FieldCentricFacingAngleProfiled alignToHub = swerve::requests::FieldCentricFacingAngleProfiled{}
        .WithCenterOfRotation({-LauncherConstants::kTurretOffset.X(), LauncherConstants::kTurretOffset.Y()})
        .WithDeadband(AlignmentConstants::kMaxSpeed * AlignmentConstants::kDeadband)
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage)
        .WithSteerRequestType(swerve::SteerRequestType::Position)
        .WithHeadingPID(AlignmentConstants::kP, AlignmentConstants::kI, AlignmentConstants::kD);
    frc::SlewRateLimiter<units::meters_per_second> alignmentXLimiter{AlignmentConstants::kAccelerationLimit};
    frc::SlewRateLimiter<units::meters_per_second> alignmentYLimiter{AlignmentConstants::kAccelerationLimit};

    units::degree_t targetYaw;
    units::degree_t yawTolerance;
    units::degree_t pitch;
    units::radians_per_second_t omega;
    units::radians_per_second_t omegaTolerance;

public:
    CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};
    IntakeRoller intakeRoller;
    IntakePivot intakePivot;
    Hopper hopper;
    Launcher launcher;
    Climber climber1{RobotMap::Climber::kClimberMotor1ID, RobotMap::Climber::kClimberLimitSwitch1ID, false};
    Climber climber2{RobotMap::Climber::kClimberMotor2ID, RobotMap::Climber::kClimberLimitSwitch2ID, true};

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
