// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/area.h>
#include <units/capacitance.h>
#include <units/charge.h>
#include <units/concentration.h>
#include <units/conductance.h>
#include <units/current.h>
#include <units/curvature.h>
#include <units/data.h>
#include <units/data_transfer_rate.h>
#include <units/density.h>
#include <units/dimensionless.h>
#include <units/energy.h>
#include <units/force.h>
#include <units/frequency.h>
#include <units/illuminance.h>
#include <units/impedance.h>
#include <units/inductance.h>
#include <units/length.h>
#include <units/luminous_flux.h>
#include <units/luminous_intensity.h>
#include <units/magnetic_field_strength.h>
#include <units/magnetic_flux.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/power.h>
#include <units/pressure.h>
#include <units/radiation.h>
#include <units/solid_angle.h>
#include <units/substance.h>
#include <units/temperature.h>
#include <units/time.h>
#include <units/torque.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/volume.h>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <pathplanner/lib/path/PathConstraints.h>
#include <pathplanner/lib/config/ModuleConfig.h>
#include <pathplanner/lib/config/RobotConfig.h>

#include <frc/RobotController.h>
#include <frc/RobotBase.h>

#include <frc/DriverStation.h>
#include <pathplanner/lib/util/FlippingUtil.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <networktables/IntegerTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include <ctre/phoenix6/swerve/SwerveDrivetrainConstants.hpp>
#include <ctre/phoenix6/swerve/SwerveModuleConstants.hpp>
#include <ctre/phoenix6/swerve/SwerveDrivetrain.hpp>

#include <frc/apriltag/AprilTagFieldLayout.h>

#include <numbers>
#include <string>
#include <array>
#include <vector>
#include <algorithm>
#include <math.h>
#include <iostream>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants
{
    inline constexpr int kDriverControllerPort = 0;
    inline constexpr int kControlBoardPort = 1;

    constexpr int kLaunchButton = 10;

    constexpr int kEjectButton = 1;

    constexpr int kIntakeSwitch = 11;
    constexpr int kIntakeTogglePositionSwitch = 12;

    constexpr int kClimberExtendSwitch = 9;
    constexpr int kClimberRetractSwitch = 8;

    constexpr int kTargetHubSwitch = 3;
    constexpr int kTargetPassSwitch = 5;
    
    constexpr int kManualIntakePivotDown = 4;
    constexpr int kManualIntakePivotUp = 2;

    constexpr int kHeightAdjusterAxis = 0;
} // namespace OperatorConstants

namespace RobotMap
{
    namespace Hopper
    {
        constexpr int kFloorMotorID = 10;
        constexpr int kFeederMotorID = 9;
    }
    namespace Intake
    {
        constexpr int kRollerMotorID = 16;
        constexpr int kPivotMotorID = 15;
        constexpr int kPivotCancoderID = 5;
    }

    namespace Launcher
    {
        constexpr int kLauncherMotor1ID = 11;
        constexpr int kLauncherMotor2ID = 12;
        constexpr int kLauncherMotor3ID = 13;
        constexpr int kActuator1ID = 0;
        constexpr int kActuator2ID = 1;
        constexpr int kDeflectorCANcoderID = 6;
    }

    namespace Climber
    {
        constexpr int kClimberMotor1ID = 17;
        constexpr int kClimberLimitSwitch1ID = 0;
        constexpr int kClimberMotor2ID = 14;
        constexpr int kClimberLimitSwitch2ID = 1;
    }
}

namespace FieldConstants
{
    constexpr frc::Translation3d kBlueHubPose{4.63_m, 4.03_m, 1.8288_m};
    constexpr frc::Translation3d kRedHubPose{11.9_m, kBlueHubPose.Y(), kBlueHubPose.Z()};
    constexpr frc::Translation3d kBluePassPose{2.5_m, kBlueHubPose.Y(), 0_m};
    constexpr frc::Translation3d kRedPassPose{14_m, kBlueHubPose.Y(), 0_m};
}

#define ADD_DEFAULT_COMMAND builder.AddStringProperty("Default Command", [this] { return GetDefaultCommand()->GetName(); }, nullptr)
#define ADD_CURRENT_COMMAND builder.AddStringProperty("Current Command", [this] { return GetCurrentCommand()->GetName(); }, nullptr)