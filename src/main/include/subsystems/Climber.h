#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include "Constants.h"

using namespace ctre::phoenix6;


class Climber : public frc2::SubsystemBase {
public:

    Climber();

    void FeedClimber();


private:
    hardware::TalonFX climberMotor{RobotMap::Climber::kClimberMotorID};

    double climberMotorSpeed = 0.5;

};