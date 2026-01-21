// Based on original work by Littleton Robotics (2021-2026)
// Source: http://github.com/Mechanical-Advantage
// 
// Modified by GRP Robotics (2026)
// Source: https://github.com/GRP-Robotics-9206
//
// -------------------------------------------------------------------------
// Original license notice:
// Copyright (c) 2021-2026 Littleton Robotics
// Use of this source code is governed by a BSD license that can be found 
// in the LICENSE file at the root directory of this project.
// -------------------------------------------------------------------------

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * IO implementation for physics sim using PhotonVision simulator.
 */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    private static VisionSystemSim visionSim;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name         The name of the camera.
     * @param poseSupplier Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonVisionSim(
        String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera);
        this.poseSupplier = poseSupplier;

        // Initialize vision sim
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(aprilTagLayout);
        }

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
        cameraSim.enableDrawWireframe(true);
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}
