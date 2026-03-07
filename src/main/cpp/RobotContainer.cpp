// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

using namespace RobotContainerConstants;

RobotContainer::RobotContainer()
{
    ConfigureBindings();

    frc2::CommandScheduler::GetInstance().Schedule(fuelUpdateCommand);
    frc2::CommandScheduler::GetInstance().Schedule(UpdateVisionMeasurementsCommand());
    frc2::CommandScheduler::GetInstance().Schedule(UpdateTargetCommand());
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

    joystick.POVUp().Debounce(60_ms).WhileTrue(frc2::cmd::Run([this] {  launcherSetSpeed += 50_rpm; }));
    joystick.POVDown().Debounce(60_ms).WhileTrue(frc2::cmd::Run([this] {  
        launcherSetSpeed -= 50_rpm; 
        if (launcherSetSpeed < 0_rpm) launcherSetSpeed = 0_rpm; }));


    controlBoard.Button(OperatorConstants::kLaunchButton).WhileTrue
    (
        GetAlignAndLaunchCommand().Unless([this] { return target == Targets::Manual; })
        .AndThen(frc2::cmd::Run([this]
        {
            launcher.SetLauncherSpeed(launcherSetSpeed);
            launcher.currentState.omega = launcherSetSpeed;
            if (launcher.IsLauncherSpeedWithinTolerance(100_rpm)) 
            {
                hopper.FeedLauncher();
            }
            else
            {
                hopper.StopMotors();
            }
            // launcher.SetLauncherPosition(4 * (controlBoardRegular.GetRawAxis(OperatorConstants::kHeightAdjusterAxis) - 0.5));
        })).WithName("Launch")
    );

    controlBoard.Button(OperatorConstants::kTargetHubSwitch).Debounce(60_ms).OnChange(UpdateTargetCommand());
    controlBoard.Button(OperatorConstants::kTargetPassSwitch).Debounce(60_ms).OnChange(UpdateTargetCommand());

    controlBoard.Button(OperatorConstants::kIntakeSwitch).WhileTrue(intakeRoller.IntakeCommand());
    controlBoard.Button(OperatorConstants::kEjectButton).WhileTrue(intakeRoller.EjectCommand());

    controlBoard.Button(OperatorConstants::kIntakeTogglePositionSwitch).WhileTrue(intakePivot.SetPositionToGroundCommand());
    controlBoard.Button(OperatorConstants::kIntakeTogglePositionSwitch).WhileFalse(intakePivot.SetPositionToHomeCommand());
    controlBoard.Button(OperatorConstants::kManualIntakePivotUp).WhileTrue(intakePivot.SetSpeedCommand(IntakePivotConstants::kManualSpeed));
    controlBoard.Button(OperatorConstants::kManualIntakePivotDown).WhileTrue(intakePivot.SetSpeedCommand(-IntakePivotConstants::kManualSpeed));

    controlBoard.Button(OperatorConstants::kClimberExtendSwitch).WhileTrue
    (   
        frc2::cmd::Parallel(climber1.ExtendClimberWithLimitCommand(), climber2.ExtendClimberWithLimitCommand()).WithName("Extend Climbers With Limits")
    );

    controlBoard.Button(OperatorConstants::kClimberRetractSwitch).WhileTrue
    (
        frc2::cmd::Parallel(climber1.RetractClimberCommand(), climber2.RetractClimberCommand()).WithName("Retract Climbers")
    );

    joystick.Y().Debounce(60_ms).OnTrue(frc2::cmd::RunOnce([this] {drivetrain.SeedFieldCentric(); }));
}

frc2::CommandPtr RobotContainer::GetAlignAndLaunchCommand()
{
    return frc2::cmd::Parallel
    (
        frc2::cmd::Run
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
                if (target == Targets::Hub)
                {
                    targetPose = frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue ? FieldConstants::kBlueHubPose : FieldConstants::kRedHubPose;
                    toleranceRadius = TargetConstants::kHubToleranceRadius;
                }
                else
                {
                    targetPose = frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue ? FieldConstants::kBluePassPose : FieldConstants::kRedPassPose;
                    toleranceRadius = TargetConstants::kPassToleranceRadius;
                }
                    
                units::meter_t deltaZ = targetPose.Z() - turretPose.Z();
                
                units::meter_t zOffset = (target == Targets::Hub ? TargetConstants::kHubZOffset : TargetConstants::kPassZOffset) + units::meter_t{GetHeightAdjustment(-0.5, 2)};
                frc::SmartDashboard::PutNumber("zOffset (ft)", zOffset.convert<units::feet>().value());

                units::standard_gravity_t g{-1};
                units::meters_per_second_t vz = units::math::sqrt(2 * (deltaZ + zOffset) * -g);
                units::second_t timeOfFlight = (-vz - units::math::sqrt(units::math::pow<2>(vz) + 2 * g * deltaZ)) / g;
                units::meters_per_second_t vx = (targetPose.X() - turretPose.X()) / timeOfFlight - turretVx;
                units::meters_per_second_t vy = (targetPose.Y() - turretPose.Y()) / timeOfFlight - turretVy;
                auto v_sq = vx*vx + vy*vy + vz*vz;
                omega = units::radians_per_second_t{sqrt(((LauncherConstants::kFuelMass * v_sq).value() / ((1 - launcher.GetLoss()) * LauncherConstants::kShooterMOI - LauncherConstants::kFuelMOIInFlywheel).value()))};
                frc::SmartDashboard::PutNumber("desiredOmega", omega.value());
                pitch = units::math::atan2(vz, units::math::hypot(vx, vy)) - robotPose.Rotation().Y();
                frc::SmartDashboard::PutNumber("desiredPitch", pitch.value());
                targetYaw = units::math::atan2(vy, vx);
                yawTolerance = units::math::atan(toleranceRadius / units::math::hypot(targetPose.X() - turretPose.X(), targetPose.Y() - turretPose.Y()));
                frc::SmartDashboard::PutNumber("targetYaw", targetYaw.value());
                frc::SmartDashboard::PutNumber("yawTolerance", yawTolerance.value());
                auto maxVr = units::math::hypot(vx, vy) + toleranceRadius / timeOfFlight;
                auto maxOmega = units::radians_per_second_t{sqrt(((LauncherConstants::kFuelMass * (maxVr*maxVr + vz*vz)).value() / ((1 - launcher.GetLoss()) * LauncherConstants::kShooterMOI - LauncherConstants::kFuelMOIInFlywheel).value()))};
                omegaTolerance = maxOmega - omega;
                frc::SmartDashboard::PutNumber("omegaTolerance", omegaTolerance.value());
                frc::SmartDashboard::PutNumber("omegaTolerance (rpm)", omegaTolerance.convert<units::revolutions_per_minute>().value());
            }
        ),
        launcher.LaunchCommand([this] { return LauncherState{pitch, omega}; }),
        drivetrain.ApplyRequest
        (
            [this]()
            {
                return alignToHub.WithTargetDirection(frc::Rotation2d{targetYaw})
                .WithTargetRateFeedforward(alignToHub.HeadingController.GetSetpoint().velocity)
                .WithVelocityX(alignmentXLimiter.Calculate(-joystick.GetLeftY() * TargetConstants::kMaxSpeed))
                .WithVelocityY(alignmentYLimiter.Calculate(-joystick.GetLeftX() * TargetConstants::kMaxSpeed)); // + joystick.GetRightX() * 1.5_mps));
            }
        ),
        intakePivot.BounceCommand().WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming),
        intakeRoller.IntakeCommand().WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming),
        hopper.FeedLauncherCommand().OnlyIf([this] { return IsAlignmentWithinTolerances(); }).Repeatedly().WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming),
        frc2::cmd::Sequence(frc2::cmd::RunOnce([this] {simFuelManager.ShootActivated(); }).OnlyIf([this] { return IsAlignmentWithinTolerances(); } ), frc2::cmd::Wait(40_ms)
                , frc2::cmd::RunOnce([this] { launcher.SimulateShootingFuel(); }), frc2::cmd::Wait(80_ms))
                .Repeatedly().OnlyIf(frc::RobotBase::IsSimulation)
    );
}

frc2::CommandPtr RobotContainer::UpdateVisionMeasurementsCommand()
{
    return frc2::cmd::Run
    (
        [this]
        {
            for (photon::PhotonCamera *camera : cameras)
            {
                for (const auto& result : camera->GetAllUnreadResults()) 
                {
                    auto visionEst = turretEstimator.EstimateCoprocMultiTagPose(result);
                    if (!visionEst) 
                    {
                        visionEst = turretEstimator.EstimateLowestAmbiguityPose(result);
                    }
                    if (visionEst)
                    {
                        drivetrain.AddVisionMeasurement(visionEst->estimatedPose.ToPose2d(), visionEst->timestamp, std::array<double, 3>{0.5, 0.5, 1});
                    }
                }
            }
        }
    ).WithName("Update Vision Measurements");
}

frc2::CommandPtr RobotContainer::UpdateTargetCommand()
{
    return frc2::cmd::RunOnce
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
        }
    ).WithName("Update Target");
}