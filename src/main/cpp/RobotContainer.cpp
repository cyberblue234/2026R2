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
	}
    autoChooser.AddOption("Center Field Right", "Center Field Right");
    autoChooser.AddOption("Center Field Right B", "Center Field Right B");

    frc2::CommandScheduler::GetInstance().Schedule(fuelUpdateCommand);
    frc2::CommandScheduler::GetInstance().Schedule(UpdateTargetCommand());
    frc2::CommandScheduler::GetInstance().Schedule(UpdateAutoShootPhysicsCommand());

    pathplanner::NamedCommands::registerCommand("Align and Shoot", AlignAndLaunch());
    pathplanner::NamedCommands::registerCommand("Shoot", frc2::cmd::Parallel
    (
        intakeRoller.IntakeCommand(),
        frc2::cmd::WaitUntil([this] { return IsAlignmentWithinTolerances(); }).AndThen(Feed()),
        launcher.LaunchCommand([this] { return LauncherState{pitch, omega}; })
    ));
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
            // auto alignToHubCommand = pathplanner::NamedCommands::getCommand("Align and Shoot");
            // if (alignToHubCommand.IsScheduled()) {
            //     return;
            // }
            drivetrain.SetControl(autonDrive.WithSpeeds(speeds));
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
}

std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand()
{
	std::string auton = autoChooser.GetSelected();
    if (auton == "Nothing") return {};
    else if (auton == "Center Field Right") return pathplanner::PathPlannerAuto("Center Field Left", true).ToPtr();
    else if (auton == "Center Field Right B") return pathplanner::PathPlannerAuto("Center Field Left B", true).ToPtr();
    return pathplanner::PathPlannerAuto(auton).ToPtr();
}

void RobotContainer::ConfigureBindings()
{
    // Configure your trigger bindings here

    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive
                .WithVelocityX(driveXLimiter.Calculate(-SquareAndPreserveSign(joystick.GetLeftY()) * DriveConstants::kMaxSpeed)) // Drive forward with negative Y (forward)
                .WithVelocityY(driveYLimiter.Calculate(-SquareAndPreserveSign(joystick.GetLeftX()) * DriveConstants::kMaxSpeed)) // Drive left with negative X (left)
                .WithRotationalRate(driveYawLimiter.Calculate(-joystick.GetRightX() * DriveConstants::kMaxAngularRate)); // Drive counterclockwise with negative X (left)
        }).WithName("Drive")
    );

    launcher.SetDefaultCommand(
        frc2::cmd::Either
        (
            launcher.StopMotorsCommand(),
            launcher.LaunchCommand([this] { return LauncherState{77_deg, 3500_rpm}; }),
            frc::DriverStation::IsTest
        ).WithName("Launcher Default")
    );

    joystick.LeftTrigger().WhileTrue
    (
        drivetrain.ApplyRequest([this]() -> auto&& {
            return driveRobotCentric
                    .WithVelocityX(driveXLimiter.Calculate(-SquareAndPreserveSign(joystick.GetLeftY()) * DriveConstants::kMaxSpeed / 3)) // Drive forward with negative Y (forward)
                    .WithVelocityY(driveYLimiter.Calculate(-SquareAndPreserveSign(joystick.GetLeftX()) * DriveConstants::kMaxSpeed / 3)) // Drive left with negative X (left)
                    .WithRotationalRate(driveYawLimiter.Calculate(-joystick.GetRightX() * DriveConstants::kMaxAngularRate / 3)); // Drive counterclockwise with negative X (left)
           }).WithName("Drive Robot Centric")
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
            AlignAndLaunch(), // Run auto launch otherwise
            [this] { return target == Targets::Manual; }
        ).WithName("Launch")
    );

    controlBoard.Button(OperatorConstants::kLaunchButton).OnFalse
    (
        frc2::cmd::Either
        (
            intakePivot.SetPositionToHomeCommand(),
            intakePivot.SetPositionToGroundCommand(),
            [this] { return controlBoardRegular.GetRawButton(OperatorConstants::kIntakeTogglePositionSwitch); }
        ).WithName("After Launch Position")
    );

    controlBoard.Button(OperatorConstants::kIntakeSwitch).WhileTrue(intakeRoller.IntakeCommand());
    controlBoard.Button(OperatorConstants::kEjectButton).WhileTrue(
        frc2::cmd::Parallel
        (
            intakeRoller.EjectCommand(), launcher.EjectCommand(), floor.EjectCommand(), feeder.EjectCommand()
        ).WithName("Eject")
    );

    controlBoard.Button(OperatorConstants::kIntakeTogglePositionSwitch).OnTrue(intakePivot.SetPositionToHomeCommand());
    controlBoard.Button(OperatorConstants::kIntakeTogglePositionSwitch).OnFalse(intakePivot.SetPositionToGroundCommand());
    controlBoard.Button(OperatorConstants::kManualIntakePivotUp).WhileTrue(intakePivot.SetSpeedCommand(-IntakeConstants::kManualSpeed));
    controlBoard.Button(OperatorConstants::kManualIntakePivotDown).WhileTrue(intakePivot.SetSpeedCommand(IntakeConstants::kManualSpeed));

    // joystick.POVUp().WhileTrue
    // (
    //     frc2::cmd::Parallel(climber1.ExtendClimberCommand()).WithName("Extend Climbers Without Limits")
    // );

    // controlBoard.Button(OperatorConstants::kClimberExtendSwitch).WhileTrue
    // (   
    //     frc2::cmd::Parallel(climber1.ExtendClimberWithLimitCommand()).WithName("Extend Climbers With Limits")
    // );

    // joystick.POVDown().WhileTrue
    // (
    //     frc2::cmd::Parallel(climber1.RetractClimberCommand()).WithName("Retract Climbers Without Limits")
    // );

    // controlBoard.Button(OperatorConstants::kClimberRetractSwitch).WhileTrue
    // (
    //     frc2::cmd::Parallel(climber1.RetractClimberWithLimitCommand()).WithName("Retract Climbers With Limits")
    // );

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
            if (target == Targets::Hub)
            {
                targetPose = alliance.value() == frc::DriverStation::Alliance::kBlue ? FieldConstants::kBlueHubPose : FieldConstants::kRedHubPose;
                toleranceRadius = TargetConstants::kHubToleranceRadius;
                zOffset = TargetConstants::kHubZOffset;
            }
            else
            {
                targetPose = alliance.value() == frc::DriverStation::Alliance::kBlue 
                        ? frc::Translation3d{FieldConstants::kBluePassPose.X(), FieldConstants::kBluePassPose.Y() + passOffset, FieldConstants::kBluePassPose.Z()} 
                        : frc::Translation3d{FieldConstants::kRedPassPose.X(), FieldConstants::kRedPassPose.Y() - passOffset, FieldConstants::kRedPassPose.Z()};
                toleranceRadius = TargetConstants::kPassToleranceRadius;
                zOffset = TargetConstants::kPassZOffset;
            }
                
            units::meter_t deltaZ = targetPose.Z() - turretPose.Z();

            units::standard_gravity_t g{-1};
            units::meters_per_second_t vz = units::math::sqrt(2 * (deltaZ + zOffset) * -g);
            units::second_t timeOfFlight = (-vz - units::math::sqrt(units::math::pow<2>(vz) + 2 * g * deltaZ)) / g;
            units::meter_t deltaX = targetPose.X() - turretPose.X();
            units::meter_t deltaY = targetPose.Y() - turretPose.Y();
            units::meters_per_second_t vx = deltaX / timeOfFlight - turretVx;
            units::meters_per_second_t vy = deltaY / timeOfFlight - turretVy;
            pitch = units::math::atan2(vz, units::math::hypot(vx, vy)) - robotPose.Rotation().Y();
            units::meter_t deltaR = units::math::hypot(deltaX, deltaY);
            timeOfFlight = units::math::sqrt((2 * (deltaR * units::math::tan(launcher.GetLauncherAngle()) - deltaZ)) / -g);
            vx = deltaX / timeOfFlight - turretVx;
            vy = deltaY / timeOfFlight - turretVy;
            vz = (deltaZ - 0.5 * g * units::math::pow<2>(timeOfFlight)) / timeOfFlight;
            auto v_sq = vx*vx + vy*vy + vz*vz;
            units::meters_per_second_t v = units::math::sqrt(v_sq);
            omega = units::revolutions_per_minute_t{(launcher.GetSpeedRatio() * v).value()}; //units::radians_per_second_t{sqrt(((LauncherConstants::kFuelMass * v_sq).value() / ((1 - launcher.GetLoss()) * LauncherConstants::kShooterMOI - LauncherConstants::kFuelMOIInFlywheel).value()))};
            frc::SmartDashboard::PutNumber("Generic/Desired Omega (rpm)", omega.convert<units::revolutions_per_minute>().value());
            
            frc::SmartDashboard::PutNumber("Generic/Desired Pitch (deg)", pitch.value());
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
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive
                .WithVelocityX(driveXLimiter.Calculate(-SquareAndPreserveSign(joystick.GetLeftY()) * DriveConstants::kMaxSpeed / 2)) // Drive forward with negative Y (forward)
                .WithVelocityY(driveYLimiter.Calculate(-SquareAndPreserveSign(joystick.GetLeftX()) * DriveConstants::kMaxSpeed / 2)) // Drive left with negative X (left)
                .WithRotationalRate(driveYawLimiter.Calculate(-joystick.GetRightX() * DriveConstants::kMaxAngularRate / 2)); // Drive counterclockwise with negative X (left)
        }),
        intakeRoller.IntakeCommand(),
        frc2::cmd::WaitUntil([this] { return launcher.IsLauncherSpeedWithinTolerance(100_rpm); }).AndThen(Feed())
    ).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
}

frc2::CommandPtr RobotContainer::AlignAndLaunch()
{
    return frc2::cmd::Parallel
    (   
        intakeRoller.IntakeCommand(),
        frc2::cmd::WaitUntil([this] { return IsAlignmentWithinTolerances(); }).AndThen(Feed()),
        launcher.LaunchCommand([this] { return LauncherState{pitch, omega}; }),
        // Driver control during teleop
        drivetrain.ApplyRequest
        (
            [this]()
            {
                return alignToHub.WithTargetDirection(frc::Rotation2d{targetYaw})
                .WithTargetRateFeedforward(alignToHub.HeadingController.GetSetpoint().velocity)
                .WithVelocityX(alignmentXLimiter.Calculate(-joystick.GetLeftY() * TargetConstants::kMaxSpeed))
                .WithVelocityY(alignmentYLimiter.Calculate(-joystick.GetLeftX() * TargetConstants::kMaxSpeed)); // + joystick.GetRightX() * 1.5_mps));
            }
        ).Until([this] { return IsAlignmentWithinTolerances() && joystick.GetRightTriggerAxis() > 0.5; })
        .AndThen(drivetrain.ApplyRequest
        (
            [this] { return brake; }
        ))
    ).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
}

frc2::CommandPtr RobotContainer::Feed()
{
    return frc2::cmd::Parallel
    (
        frc2::cmd::Wait(0.1_s).AndThen(feeder.FeedCommand()),
        frc2::cmd::Wait(0.35_s).AndThen
        (
            frc2::cmd::Parallel
            (
                floor.FeedCommand(),
                intakePivot.BounceCommand(),
                frc2::cmd::RepeatingSequence(frc2::cmd::RunOnce([this] {simFuelManager.ShootActivated(); }), frc2::cmd::Wait(80_ms)).OnlyIf(frc::RobotBase::IsSimulation)
            )
        )
    );
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