package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterCalculator;

public class ShootingCommands {
    private static final double ANGLE_KP = 3.0;
    private static final double ANGLE_KD = 0.1;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;

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
                    angleController
                );

                drive.runVelocity(aimSpeeds);
                shooter.shoot(shotData.velocity());
            },
            drive, shooter
        )
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
        .finallyDo(() -> {
            drive.stop();
            CommandScheduler.getInstance().schedule(shooter.stop());
        });
    }

    public static Command autoShoot(Drive drive, Shooter shooter) {
        return Commands.run(
            () -> {
                var shotData = ShooterCalculator.calculateMovingShot(
                    drive.getPose(), 
                    drive.getFieldRelativeSpeeds()
                );
                shooter.shoot(shotData.velocity());
            },
            shooter
        ).withName("AutoShoot");
    }

    public static Command enableSOTM(Drive drive) {
        return Commands.runOnce(() -> drive.setSOTM(true)).withName("EnableSOTM");
    }

    public static Command disableSOTM(Drive drive) {
        return Commands.runOnce(() -> drive.setSOTM(false)).withName("DisableSOTM");
    }
}