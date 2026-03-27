package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterCalculator;

public class ShootingCommands {
    /**
     * Creates a command that simultaneously aims the drivetrain toward a computed shot aim point
     * and spools the shooter to the velocity required for that moving shot.
     */
    public static Command aimAndSpool(
        Drive drive,
        Shooter shooter,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier
    ) {
        ProfiledPIDController angleController = new ProfiledPIDController(
            5.0, 0.0, 0.4, 
            new TrapezoidProfile.Constraints(8.0, 20.0)
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

                //drive.runVelocity(aimSpeeds);
                shooter.setTargetVelocity(shotData.velocity());
            },
            drive, shooter
        )
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
        .finallyDo(() -> {
            drive.stop();
            shooter.stop().schedule();
        });
    }

    public static Command autoSpool(Drive drive, Shooter shooter) {
        return Commands.run(
            () -> {
                var shotData = ShooterCalculator.calculateMovingShot(
                    drive.getPose(), 
                    drive.getFieldRelativeSpeeds()
                );
                shooter.setTargetVelocity(shotData.velocity());
            },
            shooter
        ).withName("AutoSpool");
    }

    public static Command autoKick(
        Kicker kicker,
        Shooter shooter
    ) {
        return kicker.run()
            .onlyIf(shooter::atSetpoint)
            .withName("AutoKick");
    }

    public static Command enableSOTM(Drive drive) {
        return Commands.runOnce(() -> drive.setSOTM(true)).withName("EnableSOTM");
    }

    public static Command disableSOTM(Drive drive) {
        return Commands.runOnce(() -> drive.setSOTM(false)).withName("DisableSOTM");
    }
}