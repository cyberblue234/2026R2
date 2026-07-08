// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

using namespace RobotContainerConstants;

RobotContainer::RobotContainer()
{
	// Needs to be called in the constructor of RobotContainer. Technically, all of the code found in ConfigureBindings could be put here, but it helps keep the code more organized
    ConfigureBindings();

	// This is how we put autos from Pathplanner automatically to the smart dashboard 
    std::vector<std::string> autos = pathplanner::AutoBuilder::getAllAutoNames();
    autoChooser.SetDefaultOption("Nothing", "Nothing");
	for (auto i = autos.begin(); i != autos.end(); ++i)
	{
		autoChooser.AddOption(*i, *i);
		// This was used to automatically flip left side autos to the right side without having to use the Pathplanner GUI to make both
        std::size_t leftPos = i->find("Left");
        // Only add the right version of the path if it's not a depot path, since depot paths are one-sided
        if(leftPos != std::string::npos && i->find("Depot") == std::string::npos)
        {
            i->replace(leftPos, 4, "Right");
            autoChooser.AddOption(*i, *i);
        }
	}

	// Immediately schedule some auto shoot and physics commands
    frc2::CommandScheduler::GetInstance().Schedule(fuelUpdateCommand);
    frc2::CommandScheduler::GetInstance().Schedule(UpdateTargetCommand());
    frc2::CommandScheduler::GetInstance().Schedule(UpdateAutoShootPhysicsCommand());

	// pathplanner::NamedCommands::registerCommand function is how we use commands that are defined in our code with the Pathplanner GUI. More info in the doc
    pathplanner::NamedCommands::registerCommand("Enable Vision", frc2::cmd::RunOnce([this] { visionEnabled = true; }));
    pathplanner::NamedCommands::registerCommand("Disable Vision", frc2::cmd::RunOnce([this] { visionEnabled = false; }));

    // Starts flywheel
    pathplanner::NamedCommands::registerCommand("Prepare Launcher", launcher.LaunchCommand([this] { return LauncherState{TargetConstants::kLauncherAngle, 3600_rpm}; }));
    // For some reason the default commands aren't registering during autonomous routines, so manually adding stop commands
    pathplanner::NamedCommands::registerCommand("Stop Launcher", launcher.LaunchCommand([this] { return LauncherState{TargetConstants::kLauncherAngle, 0_rpm}; }));
    pathplanner::NamedCommands::registerCommand("Stop Hopper", feeder.StopCommand().AlongWith(floor.StopCommand()));
    pathplanner::NamedCommands::registerCommand("Align and Shoot", Align().AlongWith(Launch().OnlyIf([this] { return IsAlignmentWithinTolerances();}).Repeatedly()));
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

	// this configure function is how we easily tell Pathplanner how to drive our robot
    pathplanner::AutoBuilder::configure
    (
        [this] { return drivetrain.GetState().Pose; }, // Tells Pathplanner the position of the robot
        [this](const frc::Pose2d& pose) { drivetrain.ResetPose(pose); }, // Tells Pathplanner how to reset the position
        [this] { return drivetrain.GetState().Speeds; }, // Tells Pathplanner the currents ChassisSpeeds of the robot
        [this](const frc::ChassisSpeeds& speeds, const pathplanner::DriveFeedforwards& feedforwards) {
            autonSetSpeeds = speeds;
            autonSetFeedforwards = feedforwards;
            drivetrain.SetControl(autonDrive.WithSpeeds(speeds).WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX).WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY));
        }, // Tells Pathplanner how to drive the robot
        std::make_shared<pathplanner::PPHolonomicDriveController>(
            pathplanner::PIDConstants{PathPlannerConstants::Translation::kP, PathPlannerConstants::Translation::kI, PathPlannerConstants::Translation::kD},
            pathplanner::PIDConstants{PathPlannerConstants::Rotation::kP, PathPlannerConstants::Rotation::kI, PathPlannerConstants::Rotation::kD}), 
		// Gives Pathplanner PID values for motion (Translation) and rotation. Need to be tuned for every robot. I've never gotten this to be very good.
        PathPlannerConstants::kConfig, // A config object located in the Constants.h file
        []() 
        { 
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed; // Depending on which side of the field you make the autos on, checking which Alliance this equals will change. 
																			   // This should return true when you want to flip the auto. So, if you used Pathplanner to make autos on the Blue side, you should check if alliance.value() == frc::DriverStation::Alliance::kRed. Vice versa if autos were made on Red side
            }
            return false;
        }, // Tells Pathplanner which alliance to automatically flip sides of the field
        &drivetrain
    );

	// This tells the drivetrain how much to trust the wheel encoders. Higher values = less trust
    drivetrain.SetStateStdDevs(std::array<double, 3>{0.5, 0.5, 3});
	// I don't think this ever worked but here's an idea:
    // When the Pigeon 2 detects a sudden acceleration (like from going over the bump), increase the standard deviations for 0.5 seconds to trust the wheel encoders less, then set them back to normal
    frc2::Trigger([this] { return units::math::abs(drivetrain.GetPigeon2().GetAccelerationZ().GetValue()) > 0.5_mps_sq; }).Debounce(60_ms)
        .OnTrue(
            frc2::cmd::Sequence
            (
                frc2::cmd::Print("Increasing drivetrain std devs"),
                frc2::cmd::RunOnce([this] { drivetrain.SetStateStdDevs(std::array<double, 3>{5, 5, 15}); }),
                frc2::cmd::Wait(0.5_s)
            )
        .AndThen(frc2::cmd::RunOnce([this] { drivetrain.SetStateStdDevs(std::array<double, 3>{0.5, 0.5, 3}); })));
}

std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand()
{  
	std::string auton = autoChooser.GetSelected();
    // If any autos have "Right" in the name, Pathplanner will use the "Left" version of the auto and flip it
    // No autos should be made with "Right" in the name because of this
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
    joystick.RightTrigger().WhileTrue(Align()); // Align to Hub or Pass position
    joystick.A().WhileTrue(drivetrain.ApplyRequest([this] { return brake; }).WithName("Brake")); // Set wheels to X state

    // Launching Controls
    launcher.SetDefaultCommand(
        frc2::cmd::Either
        (
            launcher.StopMotorsCommand(), // In test mode we don't want the launcher to be constantly running
            frc2::cmd::Either
            (
                // When the target is the hub, set the launcher angle to 70 degrees (TargetConstants::kLauncherAngle) and run a constant 1500 rpm
                launcher.LaunchCommand([this] { return LauncherState{TargetConstants::kLauncherAngle, 1500_rpm}; }).Until([this] { return target != Targets::Hub; }),
                // When the target is the pass, set the launcher angle to 52 degrees (TargetConstants::kPassLauncherAngle) and run a constant 1500 rpm
                launcher.LaunchCommand([this] { return LauncherState{TargetConstants::kPassLauncherAngle, 1500_rpm}; }).Until([this] { return target != Targets::Pass; }),
                [this] { return target != Targets::Pass; }
            ),
            frc::DriverStation::IsTest
        ).WithName("Launcher Default")
    );

    // Increase manual launcher set speed
    joystick.Back().Debounce(60_ms).WhileTrue(frc2::cmd::Run([this] {  launcherSetSpeed += 50_rpm; }));
    // Decrease manual launcher set speed to a min of 0 rpm
    joystick.Start().Debounce(60_ms).WhileTrue(frc2::cmd::Run([this] {  
        launcherSetSpeed -= 50_rpm; 
        if (launcherSetSpeed < 0_rpm) launcherSetSpeed = 0_rpm; }));

    // Change the pass offset, with a limit. This adjusts where the shot lands when we do a pass
    joystick.LeftBumper().Debounce(60_ms).WhileTrue(frc2::cmd::Run([this] { passOffset += 0.1_m; if (passOffset > 2_m) passOffset = 2_m; }));
    joystick.RightBumper().Debounce(60_ms).WhileTrue(frc2::cmd::Run([this] { passOffset -= 0.1_m; if (passOffset < -2_m) passOffset = -2_m; }));

    controlBoard.Button(OperatorConstants::kLaunchButton).WhileTrue
    (
        frc2::cmd::Either
        (
            ManualLaunch(), // Run manual launch if target == Manual
            Launch(), // Run auto launch otherwise
            [this] { return target == Targets::Manual; }
        ).WithName("Launch")
    ).OnFalse( // When we stop pressing the launch button:
        frc2::cmd::Parallel
        (
            // Sets the intake pivot to either the ground or up depending on the switch
            // AsProxy() means that if the commands inside of IntakePivotDefaultCommand() end, the overall larger Parallel command that all of these subsequent commands are in will not end
            // This is not default behavior due to how command based works, so it can be very important to have the AsProxy()
            // Learn more about AsProxy(): https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html#scheduling-other-commands
            IntakePivotDefaultCommand().AsProxy(),
            frc2::cmd::Either
            (
                // Either turn the intake on or off depending on the intake switch
                intakeRoller.IntakeCommand().Repeatedly().Until([this] { return !controlBoardRegular.GetRawButton(OperatorConstants::kIntakeSwitch); }),
                intakeRoller.StopMotorCommand(),
                [this] { return controlBoardRegular.GetRawButton(OperatorConstants::kIntakeSwitch); }
            ).WithName("After Launch Intake").AsProxy() // This is also AsProxy()
        ) // The idea with making the commands within this Parallel proxies is that if something else pulls the attention of one of the commands (like if the intake switch is toggled), the other command within the Parallel will still be running
    );

    // Intake Roller Controls
    // Turn on the intake
    controlBoard.Button(OperatorConstants::kIntakeSwitch).WhileTrue(intakeRoller.IntakeCommand());
    // Eject -- runs all of the eject commands in the subsystems
    controlBoard.Button(OperatorConstants::kEjectButton).WhileTrue(
        frc2::cmd::Parallel
        (
            intakeRoller.EjectCommand(), launcher.EjectCommand(), floor.EjectCommand(), feeder.EjectCommand()
        ).WithName("Eject")
    );

    // Intake Pivot Controls
    // When the switch is on, the intake goes into the robot. When it is off, it goes to the ground
    controlBoard.Button(OperatorConstants::kIntakeTogglePositionSwitch)
        .OnTrue(intakePivot.SetPositionToHomeCommand())
        .OnFalse(intakePivot.SetPositionToGroundCommand());
    // Manually set the speed of the intake pivot motor
    controlBoard.Button(OperatorConstants::kManualIntakePivotUp).WhileTrue(intakePivot.SetSpeedCommand(-IntakeConstants::kManualSpeed));
    controlBoard.Button(OperatorConstants::kManualIntakePivotDown).WhileTrue(intakePivot.SetSpeedCommand(IntakeConstants::kManualSpeed));

    // Other Controls
    // Reset the gyro
    joystick.Y().Debounce(60_ms).OnTrue(frc2::cmd::RunOnce([this] {drivetrain.SeedFieldCentric(); }));
}

frc2::CommandPtr RobotContainer::UpdateAutoShootPhysicsCommand()
{
    // This command deals with all of the targeting information. This is how we automatically aim and shoot
return frc2::cmd::Run
    (
        [this]
        {
            auto drivetrainState = drivetrain.GetState();
            frc::Pose3d robotPose = frc::Pose3d{drivetrainState.Pose}; // Gets the position of the robot
            frc::ChassisSpeeds robotSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(drivetrainState.Speeds, robotPose.Rotation().Z()); // Speed of the robot
            // The angle of the turret in field space
            units::radian_t turretTheta = robotPose.Rotation().Z() + units::math::atan2(LauncherConstants::kTurretOffset.Y(), LauncherConstants::kTurretOffset.X());
            // The radius of the center of the turret to the center of the robot
            units::meter_t kTurretRadius = units::math::hypot(LauncherConstants::kTurretOffset.Y(), LauncherConstants::kTurretOffset.X());
            // The position of the turret on the field -- used for more accurate targeting than just using the robot position
            frc::Translation3d turretPose
            {
                robotPose.X() - units::math::cos(turretTheta) * kTurretRadius,
                robotPose.Y() - units::math::sin(turretTheta) * kTurretRadius,
                robotPose.Z() + LauncherConstants::kTurretOffset.Z()
            };
            // Using the robot speed as the turret speed is a good enough approximation (and the other math didn't really work)
            units::meters_per_second_t turretVx = robotSpeeds.vx; 
            units::meters_per_second_t turretVy = robotSpeeds.vy;
            
            frc::Translation3d targetPose;
            units::meter_t toleranceRadius;
            std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();
            if (!alliance) alliance = frc::DriverStation::Alliance::kBlue; // If the alliance is not set for some reason, set it to blue
            if (target == Targets::Pass)
            {
                // Set the target to either the blue or red "PassPose", the position on the field where the fuel should land when passing
                // This includes our pass offset, set by the left and right bumpers on the driver controller
                targetPose = alliance.value() == frc::DriverStation::Alliance::kBlue 
                        ? frc::Translation3d{FieldConstants::kBluePassPose.X(), FieldConstants::kBluePassPose.Y() + passOffset, FieldConstants::kBluePassPose.Z()} 
                        : frc::Translation3d{FieldConstants::kRedPassPose.X(), FieldConstants::kRedPassPose.Y() - passOffset, FieldConstants::kRedPassPose.Z()};
                toleranceRadius = TargetConstants::kPassToleranceRadius; // Set our tolerance (high for passing)
            }
            else
            {
                // Mostly the same as in passing, but different constants. Lower tolerance and zOffset
                targetPose = alliance.value() == frc::DriverStation::Alliance::kBlue ? FieldConstants::kBlueHubPose : FieldConstants::kRedHubPose;
                toleranceRadius = TargetConstants::kHubToleranceRadius;
            }
                
            units::meter_t deltaZ = targetPose.Z() - turretPose.Z();

            units::standard_gravity_t g{-1};

            units::meter_t deltaX = targetPose.X() - turretPose.X();
            units::meter_t deltaY = targetPose.Y() - turretPose.Y();

            // The distance needed to be covered in the XY plane
            units::meter_t deltaR = units::math::hypot(deltaX, deltaY);
            // Calculate how much time the fuel will be in the air pased on our values
            units::second_t timeOfFlight = units::math::sqrt((2 * (deltaR * units::math::tan(launcher.GetLauncherAngle()) - deltaZ)) / -g);
            // Based on the time of flight, calculate the velocities in the 3 directions
            units::meters_per_second_t vx = deltaX / timeOfFlight - turretVx;
            units::meters_per_second_t vy = deltaY / timeOfFlight - turretVy;
            units::meters_per_second_t vz = (deltaZ - 0.5 * g * units::math::pow<2>(timeOfFlight)) / timeOfFlight; // Needs to include acceleration of gravity (Δx = vt + 1/2at²)
            auto v_sq = vx*vx + vy*vy + vz*vz;
            units::meters_per_second_t v = units::math::sqrt(v_sq); // Find the magnitude of the velocity --> the angle is based on the current launch angle
            omega = units::revolutions_per_minute_t{(launcher.GetSpeedRatio() * v).value()}; // Use our tuned variable "speed ratio" to determine how many RPMs the launcher wheel needs to reach to make the shot
            frc::SmartDashboard::PutNumber("Generic/Desired Omega (rpm)", omega.convert<units::revolutions_per_minute>().value());
            
            // Figure out where the robot needs to be angled based on our velocities
            targetYaw = units::math::atan2(vy, vx);
            // Find the tolerance in the yaw direction
            yawTolerance = units::math::atan(toleranceRadius / deltaR);
            frc::SmartDashboard::PutNumber("Generic/Target Yaw (deg)", targetYaw.value());
            frc::SmartDashboard::PutNumber("Generic/Yaw tolerance (deg)", yawTolerance.value());
            // Find the maximum velocity the fuel can go at while still making the shot, use to find omegaTolerance
            units::meters_per_second_t maxVr = units::math::hypot(vx, vy) + toleranceRadius / timeOfFlight;
            units::revolutions_per_minute_t maxOmega{(launcher.GetSpeedRatio() * units::math::hypot(maxVr, vz)).value()};
            omegaTolerance = maxOmega - omega;
            frc::SmartDashboard::PutNumber("Generic/Omega Tolerance (rpm)", omegaTolerance.convert<units::revolutions_per_minute>().value());
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
        launcher.LaunchCommand([this] { return LauncherState{target == Targets::Hub ? TargetConstants::kLauncherAngle : TargetConstants::kPassLauncherAngle, omega}; })
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
                frc2::cmd::Wait(1.75_s).AndThen(intakePivot.SetSpeedCommand(-0.1)),
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
