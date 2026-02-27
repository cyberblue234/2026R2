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

    frc::SmartDashboard::PutData("alignToHubController", &alignToHub.HeadingController);
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

    controlBoard.Button(OperatorConstants::kLaunchButton).WhileTrue
    (
        frc2::cmd::Parallel
        (
            frc2::cmd::Run
            (
                [this]
                {
                    auto drivetrainState = drivetrain.GetState();
                    frc::ChassisSpeeds robotSpeeds = drivetrainState.Speeds;
                    frc::Pose3d robotPose = frc::Pose3d{drivetrainState.Pose};
                    units::radian_t turretTheta = robotPose.Rotation().Z() + units::math::atan2(LauncherConstants::kTurretOffset.Y(), LauncherConstants::kTurretOffset.X());
                    units::meter_t kTurretRadius = units::math::hypot(LauncherConstants::kTurretOffset.Y(), LauncherConstants::kTurretOffset.X());
                    frc::Translation3d turretPose
                    {
                        robotPose.X() - units::math::cos(turretTheta) * kTurretRadius,
                        robotPose.Y() - units::math::sin(turretTheta) * kTurretRadius,
                        robotPose.Z() + LauncherConstants::kTurretOffset.Z()
                    };
                    units::meters_per_second_t turretVx = drivetrain.GetVelocityX(); // + units::meters_per_second_t{(robotSpeeds.omega * units::math::sin(turretTheta) * kTurretRadius).value()};
                    units::meters_per_second_t turretVy = drivetrain.GetVelocityY(); // - units::meters_per_second_t{(robotSpeeds.omega * units::math::cos(turretTheta) * kTurretRadius).value()};
                    
                    frc::Translation3d hubPose = frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue ? FieldConstants::blueHubPose : FieldConstants::redHubPose;
                    units::meter_t deltaZ = hubPose.Z() - turretPose.Z();
                    
                    units::meter_t zOffset = kZOffset + units::meter_t{controlBoardRegular.GetRawAxis(OperatorConstants::kHeightAdjusterAxis) + 0.3};
                    
                    units::standard_gravity_t g{-1};
                    units::meters_per_second_t vz = units::math::sqrt(2 * (deltaZ + zOffset) * -g);
                    units::second_t timeOfFlight = (-vz - units::math::sqrt(units::math::pow<2>(vz) + 2 * g * deltaZ)) / g;
                    units::meters_per_second_t vx = (hubPose.X() - turretPose.X()) / timeOfFlight - turretVx;
                    units::meters_per_second_t vy = (hubPose.Y() - turretPose.Y()) / timeOfFlight - turretVy;
                    units::meters_per_second_t v = units::math::sqrt(vx*vx + vy*vy + vz*vz);
                    omega = units::radians_per_second_t{sqrt(((LauncherConstants::kFuelMass * v*v).value() / ((1 - LauncherConstants::kLoss) * LauncherConstants::kShooterMOI - LauncherConstants::kFuelMOIInFlywheel).value()))};
                    pitch = units::math::atan2(vz, units::math::hypot(vx, vy)) - robotPose.Rotation().Y();
                    targetYaw = units::math::round(units::math::atan2(vy, vx).convert<units::degrees>());
                }
            ),
            launcher.LaunchCommand([this] { return LauncherState{pitch, omega}; }),
            drivetrain.ApplyRequest
            (
                [this]()
                {
                    return alignToHub.WithTargetDirection(frc::Rotation2d{targetYaw})
                    // .WithTargetRateFeedforward(alignToHub.HeadingController.GetSetpoint().velocity)
                    .WithVelocityX(DriveXAccelerationLimiter.Calculate(-joystick.GetLeftY() * 1_mps)) // Drive forward with negative Y (forward)
                    .WithVelocityY(DriveYAccelerationLimiter.Calculate(-joystick.GetLeftX() * 1_mps)); // Drive left with negative X (left)
                }
            ),
            intakePivot.BounceCommand(),
            intakeRoller.StartIntakeCommand().Repeatedly(),
            hopper.FeedLauncherCommand().OnlyIf([this] { return launcher.IsLauncherSpeedWithinTolerance() && alignToHub.HeadingController.AtGoal(); }).Repeatedly(),
            frc2::cmd::Sequence(frc2::cmd::RunOnce([this] {simFuelManager.ShootActivated(); }), frc2::cmd::Wait(40_ms)).Repeatedly().OnlyIf(frc::RobotBase::IsSimulation)
        )
    );

    controlBoard.Button(OperatorConstants::kIntakeSwitch).WhileTrue(intakeRoller.StartIntakeCommand().Repeatedly());
    controlBoard.Button(OperatorConstants::kEjectButton).WhileTrue(intakeRoller.EjectCommand());

    controlBoard.Button(OperatorConstants::kIntakeTogglePositionSwitch).WhileTrue(intakePivot.SetPositionToGroundCommand());
    controlBoard.Button(OperatorConstants::kIntakeTogglePositionSwitch).WhileFalse(intakePivot.SetPositionToHomeCommand());

    controlBoard.Button(OperatorConstants::kClimberExtendSwitch).WhileTrue
    (   
        frc2::cmd::Parallel(climber1.ExtendClimberWithLimitCommand(), climber2.ExtendClimberWithLimitCommand())
    );

    controlBoard.Button(OperatorConstants::kClimberRetractSwitch).WhileTrue
    (
        frc2::cmd::Parallel(climber1.RetractClimberCommand(), climber2.RetractClimberCommand())
    );
}

