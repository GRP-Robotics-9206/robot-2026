package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class ShooterCalculator {
    private static final InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

    private static final double DIST_NEAR = 1.0;
    private static final double DIST_MID = 2.0;
    private static final double DIST_FAR = 3.0;

    private static final LoggedTunableNumber nearVel = new LoggedTunableNumber("Shooter/NearVel", 330.0);
    private static final LoggedTunableNumber midVel = new LoggedTunableNumber("Shooter/MidVel", 410.0);
    private static final LoggedTunableNumber farVel = new LoggedTunableNumber("Shooter/FarVel", 530.0);

    private static final LoggedTunableNumber nearTOF = new LoggedTunableNumber("Shooter/NearTOF", 0.4);
    private static final LoggedTunableNumber midTOF = new LoggedTunableNumber("Shooter/MidTOF", 0.8);
    private static final LoggedTunableNumber farTOF = new LoggedTunableNumber("Shooter/FarTOF", 1.2);

    private static final Translation2d SHOOTER_OFFSET = new Translation2d(-0.25, 0.0);


    private static void updateMaps() {
        LoggedTunableNumber.ifChanged(0, () -> {
            velocityMap.clear();
            velocityMap.put(DIST_NEAR, nearVel.get());
            velocityMap.put(DIST_MID, midVel.get());
            velocityMap.put(DIST_FAR, farVel.get());

            tofMap.clear();
            tofMap.put(DIST_NEAR, nearTOF.get());
            tofMap.put(DIST_MID, midTOF.get());
            tofMap.put(DIST_FAR, farTOF.get());
            
            System.out.println("Shooter Maps Updated!");
        }, nearVel, midVel, farVel, nearTOF, midTOF, farTOF);
    }

    public record ShotData(double velocity, double timeOfFlight, Translation2d aimPoint, Translation2d shooterLocation) {}

    /**
     * Calculates the best shot while the robot is moving.
     */
    public static ShotData calculateMovingShot(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
        updateMaps();
        
        Translation2d shooterLocation = robotPose.getTranslation().plus(
            SHOOTER_OFFSET.rotateBy(robotPose.getRotation())
        );
        
        double tangentialVx = -fieldRelativeSpeeds.omegaRadiansPerSecond * SHOOTER_OFFSET.getY();
        double tangentialVy = fieldRelativeSpeeds.omegaRadiansPerSecond * SHOOTER_OFFSET.getX();
        
        double effectiveVx = fieldRelativeSpeeds.vxMetersPerSecond + tangentialVx;
        double effectiveVy = fieldRelativeSpeeds.vyMetersPerSecond + tangentialVy;

        Translation3d staticTarget = FieldConstants.Hub.getTarget();
        Translation2d currentTarget = staticTarget.toTranslation2d();
    
        double distance = shooterLocation.getDistance(currentTarget);
        double tof = tofMap.get(distance);
        Translation2d predictedTarget = currentTarget;

        // Iterative lookahead: Predict where the target is "moving" relative to the robot
        // This compensates for the ball's travel time.
        for (int i = 0; i < 4; i++) {
            double offsetX = effectiveVx * tof;
            double offsetY = effectiveVy * tof;
            
            // We subtract the robot's movement from the target's relative position
            predictedTarget = currentTarget.minus(new Translation2d(offsetX, offsetY));
            
            distance = shooterLocation.getDistance(predictedTarget);
            tof = tofMap.get(distance);
        }

        double targetVelocity = velocityMap.get(distance);

        Logger.recordOutput("Shooter/PredictedTarget", new Translation3d(predictedTarget.getX(), predictedTarget.getY(), staticTarget.getZ()));
        Logger.recordOutput("Shooter/EffectiveDistance", distance);

        return new ShotData(targetVelocity, tof, predictedTarget, shooterLocation);
    }

    public static Rotation2d getTargetRotation(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
        ShotData data = calculateMovingShot(robotPose, fieldRelativeSpeeds);
        return data.aimPoint().minus(data.shooterLocation()).getAngle();
    }

    public static double getMinTOF() {
        return 0.4;
    }

    public static double getMaxTOF() {
        return 1.2;
    }
}