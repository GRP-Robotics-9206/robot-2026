package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterCalculator;

public class ShootingCommands {
    public static Command aimAndShoot(
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

                drive.runVelocity(aimSpeeds);
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
}