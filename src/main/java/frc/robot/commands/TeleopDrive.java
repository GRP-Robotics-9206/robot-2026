package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.util.function.DoubleSupplier;

public class TeleopDrive extends Command {
  private final SwerveSubsystem m_subsystem;
  private final SwerveInputStream m_inputStream;

  /**
   * Creates a new TeleopDriveCommand.
   *
   * @param subsystem The swerve subsystem.
   * @param xSupplier The supplier for translation X (forward/backward).
   * @param ySupplier The supplier for translation Y (left/right).
   * @param omegaSupplier The supplier for rotation velocity.
   */
  public TeleopDrive(
      SwerveSubsystem subsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier
    ) {
    
    this.m_subsystem = subsystem;

    // Use the existing helper in your subsystem to create the stream
    // This applies your deadbands and alliance-relative logic
    this.m_inputStream = SwerveInputStream.of(
      m_subsystem.getSwerveDrive(), 
      xSupplier,
      ySupplier
    )
      .withControllerRotationAxis(omegaSupplier)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(OperatorConstants.TRANSLATION_SCALE)
      .scaleRotation(-OperatorConstants.ROTATION_SCALE)
      .allianceRelativeControl(true);

    // Add the subsystem as a requirement so no other command can use it simultaneously
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    // 1. Get processed speeds from the stream
    ChassisSpeeds speeds = m_inputStream.get();

    // 2. Pass them to your subsystem's setter
    // Your subsystem setter handles optimization and motor output
    m_subsystem.driveFieldOriented(speeds);
  }

  @Override
  public void end(boolean interrupted) {
  }
}