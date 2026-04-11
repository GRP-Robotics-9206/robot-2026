package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShooterCalculator.ShotData;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class ShooterCalculator {
    private static final InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

    private static final double DIST_1 = 1.0; // Near
    private static final double DIST_2 = 2.0; // Mid-Near
    private static final double DIST_3 = 3.0; // Mid
    private static final double DIST_4 = 4.5; // Mid-Far
    private static final double DIST_5 = 6.0; // Far
    private static final double DIST_MAX = 8.5; // Max

    private static final LoggedTunableNumber vel1 = new LoggedTunableNumber("Shooter/Vel1", 450.0);
    private static final LoggedTunableNumber vel2 = new LoggedTunableNumber("Shooter/Vel2", 510.0);
    private static final LoggedTunableNumber vel3 = new LoggedTunableNumber("Shooter/Vel3", 545.0);
    private static final LoggedTunableNumber vel4 = new LoggedTunableNumber("Shooter/Vel4", 630.0);
    private static final LoggedTunableNumber vel5 = new LoggedTunableNumber("Shooter/Vel5", 690.0);
    private static final LoggedTunableNumber velMax = new LoggedTunableNumber("Shooter/VelMax", 800.0);

    private static final LoggedTunableNumber tof1 = new LoggedTunableNumber("Shooter/TOF1", 0.4);
    private static final LoggedTunableNumber tof2 = new LoggedTunableNumber("Shooter/TOF2", 0.75);
    private static final LoggedTunableNumber tof3 = new LoggedTunableNumber("Shooter/TOF3", 1.1);
    private static final LoggedTunableNumber tof4 = new LoggedTunableNumber("Shooter/TOF4", 1.6);
    private static final LoggedTunableNumber tof5 = new LoggedTunableNumber("Shooter/TOF5", 2.1);
    private static final LoggedTunableNumber tofMax = new LoggedTunableNumber("Shooter/TOFMax", 2.8);

    private static final Translation2d SHOOTER_OFFSET = new Translation2d(-0.25, 0.0);


    private static void updateMaps() {
        LoggedTunableNumber.ifChanged(0, () -> {
            velocityMap.clear();
            velocityMap.put(DIST_1, vel1.get());
            velocityMap.put(DIST_2, vel2.get());
            velocityMap.put(DIST_3, vel3.get());
            velocityMap.put(DIST_4, vel4.get());
            velocityMap.put(DIST_5, vel5.get());
            velocityMap.put(DIST_MAX, velMax.get());

            tofMap.clear();
            tofMap.put(DIST_1, tof1.get());
            tofMap.put(DIST_2, tof2.get());
            tofMap.put(DIST_3, tof3.get());
            tofMap.put(DIST_4, tof4.get());
            tofMap.put(DIST_5, tof5.get());
            tofMap.put(DIST_MAX, tofMax.get());
            
            System.out.println("Shooter Interpolation Maps Updated!");
        }, vel1, vel2, vel3, vel4, vel5, velMax, tof1, tof2, tof3, tof4, tof5, tofMax);
    }

    public record ShotData(double velocity, double timeOfFlight, Translation2d aimPoint, Translation2d shooterLocation) {}

    /**
     * Calculates the best shot while the robot is moving.
     */
    public static ShotData calculateMovingShot(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d target) {
        updateMaps();
        
        Translation2d shooterLocation = robotPose.getTranslation().plus(
            SHOOTER_OFFSET.rotateBy(robotPose.getRotation())
        );
        
        // Tangential velocity compensation for offset shooter
        double tangentialVx = -fieldRelativeSpeeds.omegaRadiansPerSecond * SHOOTER_OFFSET.getY();
        double tangentialVy = fieldRelativeSpeeds.omegaRadiansPerSecond * SHOOTER_OFFSET.getX();
        
        double effectiveVx = fieldRelativeSpeeds.vxMetersPerSecond + tangentialVx;
        double effectiveVy = fieldRelativeSpeeds.vyMetersPerSecond + tangentialVy;

        double distance = shooterLocation.getDistance(target);
        double tof = tofMap.get(distance);
        Translation2d predictedTarget = target;

        for (int i = 0; i < 4; i++) {
            double offsetX = effectiveVx * tof;
            double offsetY = effectiveVy * tof;
            predictedTarget = target.minus(new Translation2d(offsetX, offsetY));
            distance = shooterLocation.getDistance(predictedTarget);
            tof = tofMap.get(distance);
        }

        double targetVelocity = velocityMap.get(distance);
        return new ShotData(targetVelocity, tof, predictedTarget, shooterLocation);
    }

    // Overload for default target (hub center)
    public static ShotData calculateMovingShot(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
        return calculateMovingShot(robotPose, fieldRelativeSpeeds, FieldConstants.Hub.getTarget().toTranslation2d());
    }

    /**
     * Calculates a shot for a stationary robot (ignores velocity).
     */
    public static ShotData calculateStaticShot(Pose2d robotPose, Translation2d target) {
        updateMaps();
        Translation2d shooterLocation = robotPose.getTranslation().plus(
            SHOOTER_OFFSET.rotateBy(robotPose.getRotation())
        );
        double distance = shooterLocation.getDistance(target);
        return new ShotData(velocityMap.get(distance), tofMap.get(distance), target, shooterLocation);
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