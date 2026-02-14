#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Launcher.h"
#include "subsystems/Hopper.h"
#include "subsystems/Intake.h"

class LaunchCommand
    : public frc2::CommandHelper<frc2::Command, LaunchCommand>
{
public:
    explicit LaunchCommand(Launcher *launcher, Hopper *hopper, Intake *intake);

	void Initialize() override;
	void Execute() override;
	void End(bool interrupted) override;
	bool IsFinished() override;

private:
    Launcher *launcher;
    Hopper *hopper;
    Intake *intake;
};