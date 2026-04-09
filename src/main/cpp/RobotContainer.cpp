// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

using namespace RobotContainerConstants;

RobotContainer::RobotContainer()
{
    ConfigureBindings();

    std::vector<std::string> autos = pathplanner::AutoBuilder::getAllAutoNames();
    autoChooser.SetDefaultOption("Nothing", "Nothing");
	for (auto i = autos.begin(); i != autos.end(); ++i)
	{
		autoChooser.AddOption(*i, *i);
        std::size_t leftPos = i->find("Left");
        // Only add the right version of the path if it's not a depot path, since depot paths are one-sided
        if(leftPos != std::string::npos && i->find("Depot") == std::string::npos)
        {
            i->replace(leftPos, 4, "Right");
            autoChooser.AddOption(*i, *i);
        }
	}

    frc2::CommandScheduler::GetInstance().Schedule(fuelUpdateCommand);
    frc2::CommandScheduler::GetInstance().Schedule(UpdateTargetCommand());
    frc2::CommandScheduler::GetInstance().Schedule(UpdateAutoShootPhysicsCommand());

    pathplanner::NamedCommands::registerCommand("Enable Vision", frc2::cmd::RunOnce([this] { visionEnabled = true; }));
    pathplanner::NamedCommands::registerCommand("Disable Vision", frc2::cmd::RunOnce([this] { visionEnabled = false; }));

    // Stars flywheel
    pathplanner::NamedCommands::registerCommand("Prepare Launcher", launcher.LaunchCommand([this] { return LauncherState{TargetConstants::kLauncherAngle, 3600_rpm}; }));
    // For some reason the default commands aren't registering during autonomous routines, so manually adding stop commands
    pathplanner::NamedCommands::registerCommand("Stop Launcher", launcher.LaunchCommand([this] { return LauncherState{TargetConstants::kLauncherAngle, 0_rpm}; }));
    pathplanner::NamedCommands::registerCommand("Stop Hopper", feeder.StopCommand().AlongWith(floor.StopCommand()));
    pathplanner::NamedCommands::registerCommand("Align and Shoot", Launch().AlongWith(Align()));
    pathplanner::NamedCommands::registerCommand("Shoot", Launch());
    pathplanner::NamedCommands::registerCommand("Clear Column", frc2::cmd::Parallel
    (
        launcher.EjectCommand(), feeder.EjectCommand()
    ));
    pathplanner::NamedCommands::registerCommand("Manual Shoot", frc2::cmd::Parallel
    (   
        launcher.LaunchCommand([this] { return LauncherState{73_deg, 3400_rpm}; }),
        intakeRoller.IntakeCommand(),
        frc2::cmd::WaitUntil([this] { return launcher.IsLauncherSpeedWithinTolerance(50_rpm); }).AndThen(Feed())
    ));

    pathplanner::NamedCommands::registerCommand("Intake", intakeRoller.IntakeCommand().AlongWith(intakePivot.SetPositionToGroundCommand()));

    pathplanner::AutoBuilder::configure
    (
        [this] { return drivetrain.GetState().Pose; },
        [this](const frc::Pose2d& pose) { drivetrain.ResetPose(pose); },
        [this] { return drivetrain.GetState().Speeds; },
        [this](const frc::ChassisSpeeds& speeds, const pathplanner::DriveFeedforwards& feedforwards) {
            autonSetSpeeds = speeds;
            autonSetFeedforwards = feedforwards;
            drivetrain.SetControl(autonDrive.WithSpeeds(speeds).WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX).WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY));
        },
        std::make_shared<pathplanner::PPHolonomicDriveController>(
            pathplanner::PIDConstants{PathPlannerConstants::Translation::kP, PathPlannerConstants::Translation::kI, PathPlannerConstants::Translation::kD},
            pathplanner::PIDConstants{PathPlannerConstants::Rotation::kP, PathPlannerConstants::Rotation::kI, PathPlannerConstants::Rotation::kD}),
        PathPlannerConstants::kConfig,
        []() 
        { 
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        &drivetrain
    );

    drivetrain.SetStateStdDevs(std::array<double, 3>{0.5, 0.5, 3});
}

std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand()
{
	std::string auton = autoChooser.GetSelected();
    std::size_t rightPos = auton.find("Right");
    if(rightPos != std::string::npos)    
    {
        // Converts the right string back to a left for PathPlannerAuto, then passes true for the mirror parameter to mirror the path
        auton.replace(rightPos, 5, "Left");
        return pathplanner::PathPlannerAuto(auton, true).ToPtr();
    }
    if (auton == "Nothing") return {};
    return pathplanner::PathPlannerAuto(auton).ToPtr();
}

void RobotContainer::ConfigureBindings()
{
    // Drivetrain Controls
    drivetrain.SetDefaultCommand
    (
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive
                .WithVelocityX(driveXLimiter.Calculate(-SquareAndPreserveSign(joystick.GetLeftY()) * DriveConstants::kMaxSpeed)) // Drive forward with negative Y (forward)
                .WithVelocityY(driveYLimiter.Calculate(-SquareAndPreserveSign(joystick.GetLeftX()) * DriveConstants::kMaxSpeed)) // Drive left with negative X (left)
                .WithRotationalRate(driveYawLimiter.Calculate(-joystick.GetRightX() * DriveConstants::kMaxAngularRate)); // Drive counterclockwise with negative X (left)
        }).WithName("Drive")
    ); // Drive field centric
    joystick.LeftTrigger().WhileTrue
    (
        drivetrain.ApplyRequest([this]() -> auto&& {
            return driveRobotCentric
                    .WithVelocityX(driveXLimiter.Calculate(-SquareAndPreserveSign(joystick.GetLeftY()) * DriveConstants::kMaxSpeed / 3)) // Drive forward with negative Y (forward)
                    .WithVelocityY(driveYLimiter.Calculate(-SquareAndPreserveSign(joystick.GetLeftX()) * DriveConstants::kMaxSpeed / 3)) // Drive left with negative X (left)
                    .WithRotationalRate(driveYawLimiter.Calculate(-joystick.GetRightX() * DriveConstants::kMaxAngularRate / 3)); // Drive counterclockwise with negative X (left)
           }).WithName("Drive Robot Centric")
    ); // Drive robot centric
    joystick.RightTrigger().WhileTrue(Align()); // Align to Hub or Pass position
    joystick.A().WhileTrue(drivetrain.ApplyRequest([this] { return brake; }).WithName("Brake")); // Set wheels to X state

    // Launching Controls
    launcher.SetDefaultCommand(
        frc2::cmd::Either
        (
            launcher.StopMotorsCommand(),
            frc2::cmd::Either
            (
                launcher.LaunchCommand([this] { return LauncherState{TargetConstants::kLauncherAngle, 2500_rpm}; }).Until([this] { return target != Targets::Hub; }),
                launcher.LaunchCommand([this] { return LauncherState{TargetConstants::kPassLauncherAngle, 2500_rpm}; }).Until([this] { return target != Targets::Pass; }),
                [this] { return target != Targets::Pass; }
            ),
            frc::DriverStation::IsTest
        ).WithName("Launcher Default")
    );

    joystick.Back().Debounce(60_ms).WhileTrue(frc2::cmd::Run([this] {  launcherSetSpeed += 50_rpm; }));
    joystick.Start().Debounce(60_ms).WhileTrue(frc2::cmd::Run([this] {  
        launcherSetSpeed -= 50_rpm; 
        if (launcherSetSpeed < 0_rpm) launcherSetSpeed = 0_rpm; }));

    joystick.LeftBumper().Debounce(60_ms).WhileTrue(frc2::cmd::Run([this] { passOffset += 0.1_m; if (passOffset > 2_m) passOffset = 2.5_m; }));
    joystick.RightBumper().Debounce(60_ms).WhileTrue(frc2::cmd::Run([this] { passOffset -= 0.1_m; if (passOffset < -2_m) passOffset = -2.5_m; }));

    controlBoard.Button(OperatorConstants::kLaunchButton).WhileTrue
    (
        frc2::cmd::Either
        (
            ManualLaunch(), // Run manual launch if target == Manual
            Launch(), // Run auto launch otherwise
            [this] { return target == Targets::Manual; }
        ).WithName("Launch")
    ).OnFalse(
        IntakePivotDefaultCommand()
    );

    // Intake Roller Controls
    controlBoard.Button(OperatorConstants::kIntakeSwitch).WhileTrue(intakeRoller.IntakeCommand());
    controlBoard.Button(OperatorConstants::kEjectButton).WhileTrue(
        frc2::cmd::Parallel
        (
            intakeRoller.EjectCommand(), launcher.EjectCommand(), floor.EjectCommand(), feeder.EjectCommand()
        ).WithName("Eject")
    );

    // Intake Pivot Controls
    controlBoard.Button(OperatorConstants::kIntakeTogglePositionSwitch)
        .OnTrue(intakePivot.SetPositionToHomeCommand())
        .OnFalse(intakePivot.SetPositionToGroundCommand());
    controlBoard.Button(OperatorConstants::kManualIntakePivotUp).WhileTrue(intakePivot.SetSpeedCommand(-IntakeConstants::kManualSpeed));
    controlBoard.Button(OperatorConstants::kManualIntakePivotDown).WhileTrue(intakePivot.SetSpeedCommand(IntakeConstants::kManualSpeed));

    // Climber Controls
    // joystick.POVUp().WhileTrue(climber.ExtendClimberCommand().OnlyIf(frc::DriverStation::IsTest)); // Extend climber without limit switch in test mode for testing purposes
    // controlBoard.Button(OperatorConstants::kClimberExtendSwitch).WhileTrue(climber.ExtendClimberWithLimitCommand());
    // joystick.POVDown().WhileTrue(climber.RetractClimberCommand().OnlyIf(frc::DriverStation::IsTest));
    // controlBoard.Button(OperatorConstants::kClimberRetractSwitch).WhileTrue(climber.RetractClimberWithLimitCommand());

    // Other Controls
    joystick.Y().Debounce(60_ms).OnTrue(frc2::cmd::RunOnce([this] {drivetrain.SeedFieldCentric(); }));
}

frc2::CommandPtr RobotContainer::UpdateAutoShootPhysicsCommand()
{
return frc2::cmd::Run
    (
        [this]
        {
            auto drivetrainState = drivetrain.GetState();
            frc::Pose3d robotPose = frc::Pose3d{drivetrainState.Pose};
            frc::ChassisSpeeds robotSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(drivetrainState.Speeds, robotPose.Rotation().Z());
            units::radian_t turretTheta = robotPose.Rotation().Z() + units::math::atan2(LauncherConstants::kTurretOffset.Y(), LauncherConstants::kTurretOffset.X());
            units::meter_t kTurretRadius = units::math::hypot(LauncherConstants::kTurretOffset.Y(), LauncherConstants::kTurretOffset.X());
            frc::Translation3d turretPose
            {
                robotPose.X() - units::math::cos(turretTheta) * kTurretRadius,
                robotPose.Y() - units::math::sin(turretTheta) * kTurretRadius,
                robotPose.Z() + LauncherConstants::kTurretOffset.Z()
            };
            units::meters_per_second_t turretVx = robotSpeeds.vx; // + units::meters_per_second_t{(drivetrain.GetVelocityYaw() * units::math::sin(turretTheta) * kTurretRadius).value()};
            units::meters_per_second_t turretVy = robotSpeeds.vy; // - units::meters_per_second_t{(drivetrain.GetVelocityYaw() * units::math::cos(turretTheta) * kTurretRadius).value()};
            
            frc::Translation3d targetPose;
            units::meter_t toleranceRadius;
            units::meter_t zOffset;
            std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();
            if (!alliance) alliance = frc::DriverStation::Alliance::kBlue;
            if (target == Targets::Pass)
            {
                targetPose = alliance.value() == frc::DriverStation::Alliance::kBlue 
                        ? frc::Translation3d{FieldConstants::kBluePassPose.X(), FieldConstants::kBluePassPose.Y() + passOffset, FieldConstants::kBluePassPose.Z()} 
                        : frc::Translation3d{FieldConstants::kRedPassPose.X(), FieldConstants::kRedPassPose.Y() - passOffset, FieldConstants::kRedPassPose.Z()};
                toleranceRadius = TargetConstants::kPassToleranceRadius;
                zOffset = TargetConstants::kPassZOffset;
            }
            else
            {
                targetPose = alliance.value() == frc::DriverStation::Alliance::kBlue ? FieldConstants::kBlueHubPose : FieldConstants::kRedHubPose;
                toleranceRadius = TargetConstants::kHubToleranceRadius;
                zOffset = TargetConstants::kHubZOffset;
            }
                
            units::meter_t deltaZ = targetPose.Z() - turretPose.Z();

            units::standard_gravity_t g{-1};
            // units::meters_per_second_t vz = units::math::sqrt(2 * (deltaZ + zOffset) * -g);
            // units::second_t timeOfFlight = (-vz - units::math::sqrt(units::math::pow<2>(vz) + 2 * g * deltaZ)) / g;
            units::meter_t deltaX = targetPose.X() - turretPose.X();
            units::meter_t deltaY = targetPose.Y() - turretPose.Y();
            // units::meters_per_second_t vx = deltaX / timeOfFlight - turretVx;
            // units::meters_per_second_t vy = deltaY / timeOfFlight - turretVy;
            // pitch = units::math::atan2(vz, units::math::hypot(vx, vy)) - robotPose.Rotation().Y();
            // frc::SmartDashboard::PutNumber("Generic/Desired Pitch (deg)", pitch.value());
            units::meter_t deltaR = units::math::hypot(deltaX, deltaY);
            units::second_t timeOfFlight = units::math::sqrt((2 * (deltaR * units::math::tan(launcher.GetLauncherAngle()) - deltaZ)) / -g);
            units::meters_per_second_t vx = deltaX / timeOfFlight - turretVx;
            units::meters_per_second_t vy = deltaY / timeOfFlight - turretVy;
            units::meters_per_second_t vz = (deltaZ - 0.5 * g * units::math::pow<2>(timeOfFlight)) / timeOfFlight;
            auto v_sq = vx*vx + vy*vy + vz*vz;
            units::meters_per_second_t v = units::math::sqrt(v_sq);
            omega = units::revolutions_per_minute_t{(launcher.GetSpeedRatio() * v).value()}; //units::radians_per_second_t{sqrt(((LauncherConstants::kFuelMass * v_sq).value() / ((1 - launcher.GetLoss()) * LauncherConstants::kShooterMOI - LauncherConstants::kFuelMOIInFlywheel).value()))};
            frc::SmartDashboard::PutNumber("Generic/Desired Omega (rpm)", omega.convert<units::revolutions_per_minute>().value());
            
            targetYaw = units::math::atan2(vy, vx);
            yawTolerance = units::math::atan(toleranceRadius / deltaR);
            frc::SmartDashboard::PutNumber("Generic/Target Yaw (deg)", targetYaw.value());
            frc::SmartDashboard::PutNumber("Generic/Yaw tolerance (deg)", yawTolerance.value());
            units::meters_per_second_t maxVr = units::math::hypot(vx, vy) + toleranceRadius / timeOfFlight;
            units::revolutions_per_minute_t maxOmega{(launcher.GetSpeedRatio() * units::math::hypot(maxVr, vz)).value()}; //{sqrt(((LauncherConstants::kFuelMass * (maxVr*maxVr + vz*vz)).value() / ((1 - launcher.GetLoss()) * LauncherConstants::kShooterMOI - LauncherConstants::kFuelMOIInFlywheel).value()))};
            omegaTolerance = maxOmega - omega;
            frc::SmartDashboard::PutNumber("Generic/Omega Tolerance (rpm)", omegaTolerance.convert<units::revolutions_per_minute>().value());
            // auto minPitch = units::math::acos(maxVr / v);
            // pitchTolerance = pitch - minPitch;
            // frc::SmartDashboard::PutNumber("Generic/Pitch Tolerance (deg)", pitchTolerance.value());
        }
    ).IgnoringDisable(true);
}

frc2::CommandPtr RobotContainer::ManualLaunch()
{
    return frc2::cmd::Parallel
    (   
        launcher.LaunchCommand([this] { return LauncherState{units::degree_t{GetHeightAdjustment(52, 80)}, launcherSetSpeed}; }),
        intakeRoller.IntakeCommand(),
        frc2::cmd::WaitUntil([this] { return launcher.IsLauncherSpeedWithinTolerance(100_rpm); }).AndThen(Feed())
    ).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
    .WithName("Manual Launch");
}

frc2::CommandPtr RobotContainer::Launch()
{
    return frc2::cmd::Parallel
    (   
        intakeRoller.IntakeCommand(),
        frc2::cmd::WaitUntil([this] { return launcher.IsLauncherSpeedWithinTolerance(omegaTolerance); }).AndThen(Feed()),
        launcher.LaunchCommand([this] { return LauncherState{target == Targets::Hub ? TargetConstants::kLauncherAngle : TargetConstants::kPassLauncherAngle, omega}; }),
        frc2::cmd::WaitUntil([this] { return IsAlignmentWithinTolerances(); })
            .AndThen(frc2::cmd::Run([this] { joystick.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0); }).WithTimeout(0.5_s)
            .AndThen(frc2::cmd::RunOnce([this] { joystick.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0); })))
    ).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
    .WithName("Launch");
}

frc2::CommandPtr RobotContainer::Align()
{
    // Driver control during teleop
    return drivetrain.ApplyRequest
    (
        [this]()
        {
            return alignToHub.WithTargetDirection(frc::Rotation2d{targetYaw})
            .WithTargetRateFeedforward(alignToHub.HeadingController.GetSetpoint().velocity)
            .WithVelocityX(alignmentXLimiter.Calculate(-joystick.GetLeftY() * TargetConstants::kMaxSpeed))
            .WithVelocityY(alignmentYLimiter.Calculate(-joystick.GetLeftX() * TargetConstants::kMaxSpeed));
        }
    ).WithName("Align");
}

frc2::CommandPtr RobotContainer::Feed()
{
    return frc2::cmd::Parallel
    (
        feeder.FeedCommand(),
        frc2::cmd::Wait(0.15_s).AndThen
        (
            frc2::cmd::Parallel
            (
                floor.FeedCommand(),
                frc2::cmd::Wait(1.5_s).AndThen(intakePivot.BounceCommand()),
                frc2::cmd::RepeatingSequence(frc2::cmd::RunOnce([this] {simFuelManager.ShootActivated(); }), frc2::cmd::Wait(80_ms)).OnlyIf(frc::RobotBase::IsSimulation)
            )
        )
    ).WithName("Feed");
}

frc2::CommandPtr RobotContainer::IntakePivotDefaultCommand()
{
    return frc2::cmd::Either
    (
        intakePivot.SetPositionToHomeCommand(),
        intakePivot.SetPositionToGroundCommand(),
        [this] { return controlBoardRegular.GetRawButton(OperatorConstants::kIntakeTogglePositionSwitch); }
    ).WithName("After Launch Position");
}

frc2::CommandPtr RobotContainer::UpdateTargetCommand()
{
    return frc2::cmd::Run
    (
        [this]
        {
            if (controlBoardRegular.GetRawButton(OperatorConstants::kTargetHubSwitch))
            {
                target = Targets::Hub;
            }
            else if (controlBoardRegular.GetRawButton(OperatorConstants::kTargetPassSwitch))
            {
                target = Targets::Pass;
            }
            else
            {
                target = Targets::Manual;
            }

            if (frc::DriverStation::IsAutonomous()) target = Targets::Hub;
        }
    ).IgnoringDisable(true);
}