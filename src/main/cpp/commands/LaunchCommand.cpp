#include "commands/LaunchCommand.h"

LaunchCommand::LaunchCommand(Launcher *launcher, Hopper *hopper, Intake *intake)
{
    this->launcher = launcher;
    this->hopper = hopper;
    this->intake = intake;
    AddRequirements({launcher, hopper, intake});
}

void LaunchCommand::Execute() 
{
    
}