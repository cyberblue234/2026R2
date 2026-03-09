#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <frc/DigitalInput.h>
#include <frc2/command/button/Trigger.h>
#include "Constants.h"

using namespace ctre::phoenix6;

namespace ClimberConstants
{
    inline constexpr units::turn_t kMaxPosition = 100_tr;
    inline constexpr double kMotorSpeed = 0.5;
}

class Climber : public frc2::SubsystemBase
{
public:
    Climber(int motorID, int limitSwitchID, bool inverted);

    void ExtendClimber();
    void RetractClimber();
    void StopClimber();

    units::turn_t GetClimberPosition()
    {
        return climberMotor.GetPosition().GetValue();
    }

    frc2::CommandPtr ResetClimberEncoderCommand();
    frc2::CommandPtr StopClimberCommand();
    frc2::CommandPtr ExtendClimberCommand();
    frc2::CommandPtr ExtendClimberWithLimitCommand();
    frc2::CommandPtr RetractClimberCommand();
    frc2::CommandPtr RetractClimberWithLimitCommand();

    void InitSendable(wpi::SendableBuilder &builder) override;

private:
    hardware::TalonFX climberMotor;
    frc::DigitalInput climberLimitSwitch;
    frc2::Trigger climberLimitSwitchTrigger{[this]
                                            { return climberLimitSwitch.Get(); }};

    double motorSpeed = ClimberConstants::kMotorSpeed;
    units::turn_t maxPosition = ClimberConstants::kMaxPosition;

    bool isRegistered = false;
};