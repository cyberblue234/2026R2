// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

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

    constexpr int kLaunchButton = 1;

    constexpr int kEjectButton = 2;
    constexpr int kAutoIntakeSwitch = 3;

    constexpr int kIntakeSwitch = 4;
    constexpr int kIntakeHomeSwitch = 5;
    constexpr int kIntakeGroundSwitch = 6;

    constexpr int kClimberExtendSwitch = 7;
    constexpr int kClimberRetractSwitch = 8;

} // namespace OperatorConstants

namespace RobotMap
{
    namespace Hopper
    {
        constexpr int kFloorMotorID = 9;
        constexpr int kFeederMotorID = 10;
    }
    namespace Intake
    {
        constexpr int kRollerMotorID = 11;
        constexpr int kPivotMotorID = 12;
        constexpr int kPivotCancoderID = 5;
    }

    namespace Launcher
    {
        constexpr int kLauncherMotor1ID = 13;
        constexpr int kLauncherMotor2ID = 14;
        constexpr int kLauncherMotor3ID = 15;
        constexpr int kActuator1ID = 0;
        constexpr int kActuator2ID = 1;
        constexpr int kDeflectorCANcoderID = 6;
    }

    namespace Climber
    {
        constexpr int kClimberMotor1ID = 16;
        constexpr int kClimberLimitSwitch1ID = 0;
        constexpr int kClimberMotor2ID = 17;
        constexpr int kClimberLimitSwitch2ID = 1;
    }
}

namespace FieldConstants
{
    
}