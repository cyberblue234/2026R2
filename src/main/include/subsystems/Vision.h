/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <functional>
#include <limits>
#include <memory>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/estimation/VisionEstimation.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include "Constants.h"

namespace VisionConstants
{
	namespace TurretCamera
	{
		constexpr frc::Transform3d kRobotToCamera{
			frc::Translation3d{-1.039_in, 0_in, 26.05_in},
			frc::Rotation3d{0_deg, -18.1_deg, 0_deg}};
		constexpr std::string_view kCameraName = "TurretCam";
	}

    namespace BackCameraLeft
	{
		constexpr frc::Transform3d kRobotToCamera{
			frc::Translation3d{-12.25_in, 1.25_in, 20.5_in},
			frc::Rotation3d{0_deg, 0_deg, 112.5_deg}};
		constexpr std::string_view kCameraName = "BackCamLeft";
	}

    namespace BackCameraRight
	{
		constexpr frc::Transform3d kRobotToCamera{
			frc::Translation3d
            {
                VisionConstants::BackCameraLeft::kRobotToCamera.X(), 
                -VisionConstants::BackCameraLeft::kRobotToCamera.Y(), 
                VisionConstants::BackCameraLeft::kRobotToCamera.Z()
            },
			frc::Rotation3d{0_deg, 0_deg, 247.5_deg}};
		constexpr std::string_view kCameraName = "BackCamRight";
	}

	inline const Eigen::Matrix<double, 3, 1> kSingleTagStdDevs{4.0, 4.0, 9999999.0};
	inline const Eigen::Matrix<double, 3, 1> kMultiTagStdDevs{0.5, 0.5, 6.0};

	const frc::AprilTagFieldLayout kTagLayout{frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark)};

}

class Vision
{
public:
	/**
	 * @param estConsumer Lamba that will accept a pose estimate and pass it to
	 * your desired SwerveDrivePoseEstimator.
	 */
	Vision(std::function<void(frc::Pose2d, units::second_t,
							  Eigen::Matrix<double, 3, 1>)>
			   estConsumer,
		   std::string_view cameraName, frc::Transform3d robotToCamera, std::function<void(frc::Pose3d, std::vector<frc::Pose3d>)> publisherConsumer)
		: photonEstimator{VisionConstants::kTagLayout, robotToCamera}, camera{cameraName}, estConsumer{estConsumer}, publisherConsumer{publisherConsumer} 
	{
		if (frc::RobotBase::IsSimulation())
		{
			visionSim = std::make_unique<photon::VisionSystemSim>("main");

			visionSim->AddAprilTags(VisionConstants::kTagLayout);

			cameraProp = std::make_unique<photon::SimCameraProperties>();

			cameraProp->SetCalibration(960, 720, frc::Rotation2d{70_deg});
			cameraProp->SetCalibError(.35, .10);
			cameraProp->SetFPS(15_Hz);
			cameraProp->SetAvgLatency(50_ms);
			cameraProp->SetLatencyStdDev(15_ms);

			cameraSim =
				std::make_shared<photon::PhotonCameraSim>(&camera, *cameraProp.get());

			visionSim->AddCamera(cameraSim.get(), robotToCamera);
			cameraSim->EnableDrawWireframe(true);
		}
	}

	photon::PhotonPipelineResult GetLatestResult() { return latestResult; }

	void Periodic()
	{
		// Run each new pipeline result through our pose estimator
		for (const auto &result : camera.GetAllUnreadResults())
		{
			// cache result and update pose estimator
			auto visionEst = photonEstimator.EstimateCoprocMultiTagPose(result);
			if (!visionEst)
			{
				visionEst = photonEstimator.EstimateLowestAmbiguityPose(result);
			}
			latestResult = result;

			// In sim only, add our vision estimate to the sim debug field
			if (frc::RobotBase::IsSimulation())
			{
				if (visionEst)
				{
					GetSimDebugField()
						.GetObject("VisionEstimation")
						->SetPose(visionEst->estimatedPose.ToPose2d());
				}
				else
				{
					GetSimDebugField().GetObject("VisionEstimation")->SetPoses({});
				}
			}

			if (visionEst)
			{
                camera.GetCameraTable()->PutNumber("Pose Strategy", visionEst->strategy);
				estConsumer(visionEst->estimatedPose.ToPose2d(), visionEst->timestamp,
							GetEstimationStdDevs(visionEst->estimatedPose.ToPose2d()));
				std::vector<frc::Pose3d> targetPoses;
				for (const auto &tgt : result.GetTargets())
				{
					auto tagPose =
						photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
					if (tagPose)
					{
						targetPoses.push_back(*tagPose);
					}
					else
					{
						targetPoses.push_back(frc::Pose3d());
					}
				}
				publisherConsumer(visionEst->estimatedPose, targetPoses);
			}
		}
	}

	Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose)
	{
		Eigen::Matrix<double, 3, 1> estStdDevs =
			VisionConstants::kSingleTagStdDevs;
		auto targets = GetLatestResult().GetTargets();
		int numTags = 0;
		units::meter_t avgDist = 0_m;
		for (const auto &tgt : targets)
		{
			auto tagPose =
				photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
			if (tagPose)
			{
				numTags++;
				avgDist += tagPose->ToPose2d().Translation().Distance(
					estimatedPose.Translation());
			}
		}
		if (numTags == 0)
		{
			return estStdDevs;
		}
		avgDist /= numTags;
		if (numTags > 1)
		{
			estStdDevs = VisionConstants::kMultiTagStdDevs;
		}
		if (numTags == 1 && avgDist > 4_m)
		{
			estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
						  std::numeric_limits<double>::max(),
						  std::numeric_limits<double>::max())
							 .finished();
		}
		else
		{
			estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
		}
		return estStdDevs;
	}

	void SimPeriodic(frc::Pose2d robotSimPose)
	{
		visionSim->Update(robotSimPose);
	}

	void ResetSimPose(frc::Pose2d pose)
	{
		if (frc::RobotBase::IsSimulation())
		{
			visionSim->ResetRobotPose(pose);
		}
	}

	frc::Field2d &GetSimDebugField() { return visionSim->GetDebugField(); }

private:
	photon::PhotonPoseEstimator photonEstimator;
	photon::PhotonCamera camera;
	std::unique_ptr<photon::VisionSystemSim> visionSim;
	std::unique_ptr<photon::SimCameraProperties> cameraProp;
	std::shared_ptr<photon::PhotonCameraSim> cameraSim;

	// The most recent result, cached for calculating std devs
	photon::PhotonPipelineResult latestResult;
	std::function<void(frc::Pose2d, units::second_t, Eigen::Matrix<double, 3, 1>)>
		estConsumer;

	std::function<void(frc::Pose3d, std::vector<frc::Pose3d>)> publisherConsumer;
};