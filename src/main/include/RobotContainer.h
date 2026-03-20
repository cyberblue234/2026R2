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

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/Intake.h"
#include "subsystems/Hopper.h"
#include "subsystems/Launcher.h"
#include "subsystems/Climber.h"
#include "subsystems/Vision.h"

#include "sim/Fuel.hpp"

namespace RobotContainerConstants
{
    namespace DriveConstants
    {
        inline constexpr units::meters_per_second_t kMaxSpeed = TunerConstants::kSpeedAt12Volts;
        inline constexpr units::radians_per_second_t kMaxAngularRate = 1_tps;

        inline constexpr units::meters_per_second_squared_t kAccelerationLimit = 15_mps_sq;
        inline constexpr units::degrees_per_second_squared_t kAngularAccelerationLimit = 4_tr_per_s_sq;
        inline constexpr double kDeadband = 0.15;
    }

    namespace TargetConstants
    {
        inline constexpr double kP = 4;
        inline constexpr double kI = 30;
        inline constexpr double kD = 0;

        inline constexpr units::meters_per_second_t kMaxSpeed = 1.5_mps;
        inline constexpr units::meters_per_second_squared_t kAccelerationLimit = 3_mps_sq;
        inline constexpr double kDeadband = 0.2;

        inline constexpr units::meter_t kHubToleranceRadius = 16_in;
        inline constexpr units::meter_t kPassToleranceRadius = 1.5_m;
        inline constexpr units::meter_t kHubZOffset = 1.25_m;
        inline constexpr units::meter_t kPassZOffset = 4_m;
    }

    namespace IntakePivotConstants
    {
        inline constexpr double kManualSpeed = 0.2;
    }

    namespace PathPlannerConstants
    {
        inline const pathplanner::ModuleConfig kModuleConfig
        {
            TunerConstants::kWheelRadius, 
            DriveConstants::kMaxSpeed, 
            RobotConstants::kWheelCOF, 
            frc::DCMotor::KrakenX60(1), 
            TunerConstants::kDriveGearRatio,
            TunerConstants::kSlipCurrent,
            1
        };
        inline const pathplanner::RobotConfig kConfig
        {
            RobotConstants::kMass, 
            RobotConstants::kMOI, 
            kModuleConfig, 
            std::vector<frc::Translation2d>{ 
                {TunerConstants::FrontLeft.LocationX, TunerConstants::FrontLeft.LocationY}, 
                {TunerConstants::FrontRight.LocationX, TunerConstants::FrontRight.LocationY}, 
                {TunerConstants::BackLeft.LocationX, TunerConstants::BackLeft.LocationY}, 
                {TunerConstants::BackRight.LocationX, TunerConstants::BackRight.LocationY}
            }
        };

        namespace Translation
        {
            constexpr double kP = 2.0;
            constexpr double kI = 0.0;
            constexpr double kD = 0.1;
        }
        namespace Rotation
        {
            constexpr double kP = 2.0;
            constexpr double kI = 0.0;
            constexpr double kD = 0.1;
        }
    }
}

enum Targets
{
    Hub, Pass, Manual
};

using namespace RobotContainerConstants;

class RobotContainer
{
public:
    RobotContainer();

    bool IsAlignmentWithinTolerances()
    {
        return launcher.IsLauncherSpeedWithinTolerance(omegaTolerance) 
            && units::math::abs(frc::Rotation2d(targetYaw).Degrees() - drivetrain.GetState().Pose.Rotation().Degrees()) < yawTolerance;
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
        .WithDriveRequestType(swerve::DriveRequestType::Velocity);
    swerve::requests::RobotCentric driveRobotCentric = swerve::requests::RobotCentric{}
        .WithDeadband(DriveConstants::kMaxSpeed * DriveConstants::kDeadband / 3)
        .WithRotationalDeadband(DriveConstants::kMaxAngularRate * DriveConstants::kDeadband / 3)
        .WithDriveRequestType(swerve::DriveRequestType::Velocity);
    frc::SlewRateLimiter<units::meters_per_second> driveXLimiter{DriveConstants::kAccelerationLimit};
    frc::SlewRateLimiter<units::meters_per_second> driveYLimiter{DriveConstants::kAccelerationLimit};
    frc::SlewRateLimiter<units::degrees_per_second> driveYawLimiter{DriveConstants::kAngularAccelerationLimit};

    swerve::requests::FieldCentricFacingAngleProfiled alignToHub = swerve::requests::FieldCentricFacingAngleProfiled{}
        .WithCenterOfRotation({-LauncherConstants::kTurretOffset.X(), LauncherConstants::kTurretOffset.Y()})
        .WithDeadband(TargetConstants::kMaxSpeed * TargetConstants::kDeadband)
        .WithDriveRequestType(swerve::DriveRequestType::Velocity)
        .WithSteerRequestType(swerve::SteerRequestType::Position)
        .WithHeadingPID(TargetConstants::kP, TargetConstants::kI, TargetConstants::kD);
    frc::SlewRateLimiter<units::meters_per_second> alignmentXLimiter{TargetConstants::kAccelerationLimit};
    frc::SlewRateLimiter<units::meters_per_second> alignmentYLimiter{TargetConstants::kAccelerationLimit};

    swerve::requests::ApplyRobotSpeeds autonDrive = swerve::requests::ApplyRobotSpeeds{}
        .WithDriveRequestType(swerve::DriveRequestType::Velocity)
        .WithSteerRequestType(swerve::SteerRequestType::Position);
    frc::ChassisSpeeds autonSetSpeeds;
    pathplanner::DriveFeedforwards autonSetFeedforwards;

    units::degree_t targetYaw;
    units::degree_t yawTolerance;
    units::degree_t pitch;
    units::degree_t pitchTolerance;
    units::radians_per_second_t omega;
    units::radians_per_second_t omegaTolerance;

    units::revolutions_per_minute_t launcherSetSpeed = 3400_rpm;

    units::meter_t passOffset = 0_m;

    Targets target;

    CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};
    IntakeRoller intakeRoller;
    IntakePivot intakePivot;
    Floor floor;
    Feeder feeder;
    Launcher launcher;
    Climber climber1{RobotMap::Climber::kClimberMotor1ID, RobotMap::Climber::kClimberLimitSwitch1ID, true};
    Climber climber2{RobotMap::Climber::kClimberMotor2ID, RobotMap::Climber::kClimberLimitSwitch2ID, false};

    std::function<void(frc::Pose2d pose, units::second_t timestamp,
                          Eigen::Matrix<double, 3, 1> stddevs)> visionMeasurementConsumer =  
            [=, this](frc::Pose2d pose, units::second_t timestamp,
                          Eigen::Matrix<double, 3, 1> stddevs) 
            {
                if (frc::RobotBase::IsReal())
                {
                    std::array<double, 3> stddevsArr{stddevs(0), stddevs(1), stddevs(2)};
                    drivetrain.AddVisionMeasurement(pose, timestamp, stddevsArr);
                }
            };

    frc::Pose3d turretVisionPose;
    std::vector<frc::Pose3d> turretVisionTargets;
    Vision turretVision
    {
        visionMeasurementConsumer, VisionConstants::TurretCamera::kCameraName, VisionConstants::TurretCamera::kRobotToCamera,
        [=, this](frc::Pose3d pose, std::vector<frc::Pose3d> targets)
        {
            turretVisionPose = pose;
            turretVisionTargets = targets;
        }
    };

    frc::Pose3d backCameraLeftPose;
    std::vector<frc::Pose3d> backCameraLeftTargets;
    Vision backCameraLeftVision
    {
        visionMeasurementConsumer, VisionConstants::BackCameraLeft::kCameraName, VisionConstants::BackCameraLeft::kRobotToCamera,
        [=, this](frc::Pose3d pose, std::vector<frc::Pose3d> targets)
        {
            backCameraLeftPose = pose;
            backCameraLeftTargets = targets;
        }
    };

    frc::Pose3d backCameraRightPose;
    std::vector<frc::Pose3d> backCameraRightTargets;
    Vision backCameraRightVision
    {
        visionMeasurementConsumer, VisionConstants::BackCameraRight::kCameraName, VisionConstants::BackCameraRight::kRobotToCamera,
        [=, this](frc::Pose3d pose, std::vector<frc::Pose3d> targets)
        {
            backCameraRightPose = pose;
            backCameraRightTargets = targets;
        }
    };

    

    FuelManager simFuelManager;
    frc2::CommandPtr fuelUpdateCommand = simFuelManager.UpdateFuel
    (
        [this] { return frc::Pose3d{drivetrain.GetState().Pose}; }, 
        [this] { return frc::ChassisSpeeds{drivetrain.GetVelocityX(), drivetrain.GetVelocityY(), drivetrain.GetVelocityYaw()}; }, 
        launcher.GetLauncherOmegaSupplier(), 
        launcher.GetLauncherAngleAsSupplier(),
        [this] { return launcher.GetLoss(); }
    );

    std::optional<frc2::CommandPtr> GetAutonomousCommand();
    frc::SendableChooser<std::string> autoChooser;

    void ConfigureBindings();

private:
    frc2::CommandPtr UpdateAutoShootPhysicsCommand();
    frc2::CommandPtr ManualLaunch();
    frc2::CommandPtr AlignAndLaunch();
    frc2::CommandPtr Feed();

    frc2::CommandPtr UpdateTargetCommand();

    std::function<bool()> GetIntakeSwitchSupplier()
    {
        return [this] { return controlBoardRegular.GetRawButton(OperatorConstants::kIntakeSwitch); };
    }

    double SquareAndPreserveSign(double input)
    {
        return copysign(input*input, input);
    }

    double GetHeightAdjustment(double min, double max)
    {
        double test = controlBoardRegular.GetRawAxis(OperatorConstants::kHeightAdjusterAxis) * (max - min) / 2 + (max + min) / 2;
        frc::SmartDashboard::PutNumber("Height Adjustment", test);
        return test;
    }
};
