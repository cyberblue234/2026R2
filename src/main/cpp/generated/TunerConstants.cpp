#include "generated/TunerConstants.h"
#include "subsystems/CommandSwerveDrivetrain.h"

CommandSwerveDrivetrain TunerConstants::CreateDrivetrain()
{
    return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}
