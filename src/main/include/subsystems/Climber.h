#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkFlex.h>
#include <frc/DigitalInput.h>
#include <frc2/command/button/Trigger.h>
#include "Constants.h"

using namespace ctre::phoenix6;

namespace ClimberConstants
{
    inline constexpr units::turn_t kMaxPosition = 80_tr;
    inline constexpr double kMotorSpeed = 0.5;
}

class Climber : public frc2::SubsystemBase
{
public:
    Climber();

    void ExtendClimber();
    void RetractClimber();
    void StopClimber();

    units::turn_t GetClimberPosition()
    {
        return units::turn_t{motor.GetEncoder().GetPosition()};
    }

    frc2::CommandPtr ResetClimberEncoderCommand();
    frc2::CommandPtr StopClimberCommand();
    frc2::CommandPtr ExtendClimberCommand();
    frc2::CommandPtr ExtendClimberWithLimitCommand();
    frc2::CommandPtr RetractClimberCommand();
    frc2::CommandPtr RetractClimberWithLimitCommand();

    void InitSendable(wpi::SendableBuilder &builder) override;

    bool GetLimitSwitches()
    {
        return climberLimitSwitch1.Get() || climberLimitSwitch2.Get();
    }

private:
    rev::spark::SparkFlex motor{RobotMap::Climber::kClimberMotorID, rev::spark::SparkFlex::MotorType::kBrushless};
    frc::DigitalInput climberLimitSwitch1{RobotMap::Climber::kClimberLimitSwitch1ID};
    frc::DigitalInput climberLimitSwitch2{RobotMap::Climber::kClimberLimitSwitch2ID};
    frc2::Trigger climberLimitSwitchTrigger{[this]
                                            { return GetLimitSwitches(); }};

    double motorSpeed = ClimberConstants::kMotorSpeed;
    units::turn_t maxPosition = ClimberConstants::kMaxPosition;

    bool isRegistered = false;
};