package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import swervelib.SwerveInputStream;

public class AimAtPose extends Command {
  private final SwerveSubsystem drivebase;
  private final SwerveInputStream inputStream;
  private final Pose2d targetPose;
  private final PIDController rotationPid = new PIDController(5.0, 0, 0.1); 
  private final double velocityCompensationFactor = 0.05;
  private final double maxRotationVel = 4.0; 

  public AimAtPose(SwerveSubsystem drivebase, DoubleSupplier vX, DoubleSupplier vY, Pose2d targetPose) {
    this.drivebase = drivebase;
    this.targetPose = targetPose;

    // Standard YAGSL input setup
    this.inputStream = SwerveInputStream.of(drivebase.getSwerveDrive(), vX, vY)
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(OperatorConstants.TRANSLATION_SCALE)
        .allianceRelativeControl(true);

    rotationPid.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivebase);
  }

  @Override
  public void execute() {
    // 1. Get driver translation inputs
    ChassisSpeeds driverSpeeds = inputStream.get();
    
    // 2. Get current robot state
    Pose2d currentPose = drivebase.getSwerveDrive().getPose();
    ChassisSpeeds robotVel = drivebase.getSwerveDrive().getRobotVelocity();

    // 3. Compensate for robot movement (The 4481 Secret Sauce)
    Translation2d deltaPos = new Translation2d(
        robotVel.vxMetersPerSecond * velocityCompensationFactor,
        robotVel.vyMetersPerSecond * velocityCompensationFactor
    );
    Pose2d correctedPose = new Pose2d(currentPose.getTranslation().plus(deltaPos), currentPose.getRotation());

    // 4. Calculate target angle
    Translation2d deltaTranslation = targetPose.getTranslation().minus(correctedPose.getTranslation());
    Rotation2d targetRotation = deltaTranslation.getAngle();

    // 5. Calculate rotation output using PID
    double rotationOutput = rotationPid.calculate(
        currentPose.getRotation().getRadians(), 
        targetRotation.getRadians()
    );

    // 6. Optional: Add Feedforward (Helps rotation stay locked while strafing)
    // Simplified version of the 4481 feedforward
    double feedforward = Math.atan2(robotVel.vyMetersPerSecond, deltaTranslation.getNorm()) * 0.1;
    rotationOutput += feedforward;

    // 7. Clamp and Drive
    rotationOutput = MathUtil.clamp(rotationOutput, -maxRotationVel, maxRotationVel);

    drivebase.driveFieldOriented(new ChassisSpeeds(
        driverSpeeds.vxMetersPerSecond,
        driverSpeeds.vyMetersPerSecond,
        rotationOutput
    ));
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.driveFieldOriented(new ChassisSpeeds());
  }
}