#include "Constants.h"
#include "subsystems/Launcher.h"
namespace frc
{
struct Velocity3d
{
    units::meters_per_second_t vx;
    units::meters_per_second_t vy;
    units::meters_per_second_t vz;

    Velocity3d() {}

    Velocity3d(units::meters_per_second_t vx, units::meters_per_second_t vy, units::meters_per_second_t vz)
    {
        this->vx = vx;
        this->vy = vy;
        this->vz = vz;
    }

    void Transform(frc::Pose3d &pose, units::second_t dt)
    {
        units::meter_t dx = vx * dt;
        units::meter_t dy = vy * dt;
        units::meter_t dz = vz * dt - units::standard_gravity_t{1} * dt * dt;
        pose = frc::Pose3d{pose.X() + dx, pose.Y() + dy, pose.Z() + dz, pose.Rotation()};
    }

    units::meters_per_second_t GetVelocity()
    {
        return units::math::sqrt(units::math::pow<2>(vx) + units::math::pow<2>(vy) + units::math::pow<2>(vz));
    }
};
}

class Fuel
{
public:
    Fuel(frc::Pose3d pose, frc::Velocity3d velocity)
    {
        this->pose = pose;
        this->velocity = velocity;
    }

    frc::Pose3d UpdatePhysics(units::second_t dt)
    {
        velocity.Transform(pose, dt);
        velocity.vz -= units::standard_gravity_t{1} * dt;
        if (pose.Z() < 0_m) pose = frc::Pose3d{};
        return pose;
    }

private:
    frc::Pose3d pose;
    frc::Velocity3d velocity;
};


class FuelManager
{
public:
    void InstantiateFuel(frc::Pose3d pose, frc::Velocity3d velocity)
    {
        fuelContainer.push_back(Fuel{pose, velocity});
    }

    frc2::CommandPtr InstantiateFuelCommand()
    {
        return frc2::cmd::RunOnce([this] { InstantiateFuel(frc::Pose3d{}, frc::Velocity3d{}); }).IgnoringDisable(true);
    }

    void ShootActivated()
    {
        isShooting = true;
    }

    frc2::CommandPtr UpdateFuel(std::function<frc::Pose3d()> drivePose, std::function<frc::ChassisSpeeds()> driveSpeeds, std::function<units::radians_per_second_t()> shooterOmega, std::function<units::degree_t()> deflectorAngle)
    {
        return frc2::cmd::Run([this, drivePose, driveSpeeds, shooterOmega, deflectorAngle]
        {
            if (isShooting)
            {
                isShooting = false;
                units::meters_per_second_t rvx = driveSpeeds().vx;
                units::meters_per_second_t rvy = driveSpeeds().vy;
                frc::Pose3d rPose = drivePose();
                            
                units::meters_per_second_t v
                {
                    sqrt
                    ((
                        (
                            units::math::pow<2>(shooterOmega()) 
                            * ((1 - LauncherConstants::kLoss) * LauncherConstants::kShooterMOI - LauncherConstants::kFuelMOIInFlywheel)
                        ) 
                        / LauncherConstants::kFuelMass
                    ).value())
                };
                frc::SmartDashboard::PutNumber("v", v.value());
                auto vx = rvx + v * units::math::cos(deflectorAngle()) * units::math::cos(rPose.Rotation().Z());
                auto vy = rvy + v * units::math::cos(deflectorAngle()) * units::math::sin(rPose.Rotation().Z());
                auto vz = v * units::math::sin(deflectorAngle());
                frc::SmartDashboard::PutNumber("vx", vx.value());
                frc::SmartDashboard::PutNumber("vy", vy.value());
                frc::SmartDashboard::PutNumber("vz", vz.value());
                units::radian_t turretTheta = rPose.Rotation().Z() + units::math::atan2(LauncherConstants::kTurretOffset.Y(), LauncherConstants::kTurretOffset.X());
                units::meter_t kTurretRadius = units::math::hypot(LauncherConstants::kTurretOffset.Y(), LauncherConstants::kTurretOffset.X());
                frc::Pose3d startPose
                {
                    rPose.X() - units::math::cos(turretTheta) * kTurretRadius,
                    rPose.Y() - units::math::sin(turretTheta) * kTurretRadius,
                    rPose.Z() + LauncherConstants::kTurretOffset.Z(),
                    frc::Rotation3d{}
                };

                InstantiateFuel(startPose, frc::Velocity3d{vx, vy, vz});
            }
            units::second_t newTime = frc::Timer::GetFPGATimestamp();
            units::second_t dt = newTime - time;
            time = newTime;
            std::vector<frc::Pose3d> poses;
            int locationInHopper = 0;
            for (std::vector<Fuel>::iterator i = fuelContainer.begin(); i != fuelContainer.end();)
            {  
                Fuel &fuel = *i;

                const frc::Pose3d &pose = fuel.UpdatePhysics(dt);
                if (pose == frc::Pose3d{})
                {
                    i = fuelContainer.erase(i);
                    continue;
                }
                poses.push_back(pose);
                ++i;
                locationInHopper++;
            }
            fuelPublisher.Set(poses);
           
        }).IgnoringDisable(true);
    }    

private:
    std::vector<Fuel> fuelContainer;
    nt::StructArrayPublisher<frc::Pose3d> fuelPublisher = nt::NetworkTableInstance::GetDefault().GetTable("Sim")->GetStructArrayTopic<frc::Pose3d>("fuelContainer").Publish();
    bool isShooting = false;
    units::second_t time = frc::Timer::GetFPGATimestamp();
};