package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;
import yams.mechanisms.swerve.SwerveDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
    public VisionSystemSim visionSim;

    public final Camera arducam;
    public final Camera thrifty;

    Vision() {
        visionSim = new VisionSystemSim("Main");

        arducam = new Camera("Arducam", kRobotToArducam );
        thrifty = new Camera("Thrifty", kRobotToThrifty);

        setupSimulation();
    }

    public static class Camera {
        public final Transform3d robotToCamera;
        public final PhotonCamera camera;
        public final PhotonPoseEstimator estimator;

        public Camera(String name, Transform3d robotToCamera) {
            this.robotToCamera = robotToCamera;
            this.camera = new PhotonCamera(name);
            this.estimator = new PhotonPoseEstimator(kTagLayout, robotToCamera);
        }

        public void addToSim(SimCameraProperties prop, VisionSystemSim visionSim) {
            PhotonCameraSim cameraSim = new PhotonCameraSim(camera, prop);
            visionSim.addCamera(cameraSim, robotToCamera);
            cameraSim.enableDrawWireframe(true);
        }

        public void addToSim(VisionSystemSim visionSim) {
            PhotonCameraSim cameraSim = new PhotonCameraSim(camera);
            visionSim.addCamera(cameraSim, robotToCamera);
            cameraSim.enableDrawWireframe(true);
        }

        public List<CameraEstimate> getLatestEstimates() {
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            List<CameraEstimate> estimates = new ArrayList<>();

            for (PhotonPipelineResult result : results) {
                // Try Multi-tag
                Optional<EstimatedRobotPose> visionEst = estimator.estimateCoprocMultiTagPose(result);

                // Fallback to Single-tag
                if (visionEst.isEmpty()) {
                    visionEst = estimator.estimateLowestAmbiguityPose(result);
                }

                if (visionEst.isPresent()) {
                    EstimatedRobotPose est = visionEst.get();
                    Matrix<N3, N1> stdDevs = calculateStdDevs(est, result.getTargets());
                    estimates.add(new CameraEstimate(est, stdDevs));
                }
            }
            return estimates;
        }

        private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose est, List<PhotonTrackedTarget> targets) {
            int numTags = 0;
            double avgDist = 0.0;

            for (PhotonTrackedTarget tgt : targets) {
                var tagPose = kTagLayout.getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;

                numTags++;
                avgDist += tagPose.get().toPose2d().getTranslation().getDistance(est.estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) return kSingleTagStdDevs;

            avgDist /= numTags;
            Matrix<N3, N1> baseStdDevs = (numTags > 1) ? kMultiTagStdDevs : kSingleTagStdDevs;

            return baseStdDevs.times(1.0 + (avgDist * avgDist / 30.0));
        }

        public void addVisionMeasurementToSwerve(SwerveDrive swerve) {
            for (CameraEstimate est : getLatestEstimates()) {
                swerve.addVisionMeasurement(
                        est.pose.estimatedPose.toPose2d(),
                        est.pose.timestampSeconds,
                        est.stdDevs
                );
            }
        }
    }

    public record CameraEstimate(EstimatedRobotPose pose, Matrix<N3, N1> stdDevs) {}

    public void setupSimulation() {
        if (!Robot.isSimulation()) return;

        visionSim.addAprilTags(kTagLayout);
        arducam.addToSim(visionSim);
        thrifty.addToSim(visionSim);
    }

    public void updatePoseEst(SwerveDrive swerve) {
        if (RobotBase.isSimulation()) {
            visionSim.update(swerve.getPose());
        }

        arducam.addVisionMeasurementToSwerve(swerve);
        thrifty.addVisionMeasurementToSwerve(swerve);
    }
}