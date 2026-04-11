package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterCalculator;

public class ShootingCommands {
    private static final double ANGLE_KP = 3.0;
    private static final double ANGLE_KD = 0.1;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double HUB_SCORE_VELOCITY = 450.0; 
    private static final double SHOOTING_SPEED_MULTIPLIER = 0.5;

    /**
     * Creates a command that simultaneously aims the drivetrain toward a computed shot aim point
     * and spools the shooter to the velocity required for that moving shot.
     */
    public static Command aimAndShoot(
        Drive drive,
        Shooter shooter,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier
    ) {
        ProfiledPIDController angleController =
            new ProfiledPIDController(
                ANGLE_KP,
                0.0,
                ANGLE_KD,
                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION)
            );
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.run(
            () -> {
                var shotData = ShooterCalculator.calculateMovingShot(
                    drive.getPose(), 
                    drive.getFieldRelativeSpeeds()
                );

                ChassisSpeeds aimSpeeds = DriveCommands.calculateAimSpeeds(
                    drive, 
                    xSupplier, 
                    ySupplier, 
                    shotData.aimPoint(), 
                    angleController,
                    SHOOTING_SPEED_MULTIPLIER
                );

                drive.runVelocity(aimSpeeds);
                shooter.setTargetVelocity(shotData.velocity());
            },
            drive, shooter
        )
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
        .finallyDo(() -> {
            drive.stop();
            CommandScheduler.getInstance().schedule(shooter.stop());
        });
    }

    /**
     * Shoots to the center of the Alliance Zone. 
     * Note: This only sets the shooter RPM; it doesn't fight the driver for rotation.
     */
    public static Command passToAllianceZone(Drive drive, Shooter shooter) {
        return Commands.run(() -> {
            double allianceZoneX = FieldConstants.isRed() 
                ? FieldConstants.FIELD_LENGTH.minus(FieldConstants.ALLIANCE_ZONE.div(2)).in(Meters)
                : FieldConstants.ALLIANCE_ZONE.div(2).in(Meters);
            
            Translation2d passTarget = new Translation2d(allianceZoneX, FieldConstants.FIELD_WIDTH.div(2).in(Meters));

            // Use the moving calculator to get the right RPM, but we won't use the aimPoint for rotation
            var shotData = ShooterCalculator.calculateMovingShot(
                drive.getPose(), 
                drive.getFieldRelativeSpeeds(),
                passTarget
            );

            shooter.setTargetVelocity(shotData.velocity());
        }, shooter);
    }

    /**
     * Fires the shooter based on distance to Hub, but doesn't move the drivetrain.
     * Perfect for shooting while the driver is manually aiming.
     */
    public static Command shootStationary(Drive drive, Shooter shooter) {
        return Commands.run(() -> {
            var shotData = ShooterCalculator.calculateStaticShot(
                drive.getPose(), 
                FieldConstants.Hub.getTarget().toTranslation2d()
            );
            shooter.setTargetVelocity(shotData.velocity());
        }, shooter);
    }

    public static Command autoShoot(Drive drive, Shooter shooter) {
        return Commands.run(
            () -> {
                var shotData = ShooterCalculator.calculateMovingShot(
                    drive.getPose(), 
                    drive.getFieldRelativeSpeeds()
                );
                shooter.setTargetVelocity(shotData.velocity());
            },
            shooter
        ).withName("AutoShoot");
    }

    // A simple command to shoot at a fixed velocity, useful for testing and shooting directly in front of the hub.
    public static Command hubShoot(Shooter shooter) {
        return shooter.shoot(HUB_SCORE_VELOCITY);
    }

    public static Command enableSOTM(Drive drive) {
        return Commands.runOnce(() -> drive.setSOTM(true)).withName("EnableSOTM");
    }

    public static Command disableSOTM(Drive drive) {
        return Commands.runOnce(() -> drive.setSOTM(false)).withName("DisableSOTM");
    }
}