package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class AimAtGoal extends Command {
  private final SwerveSubsystem drivebase;
  private final CommandXboxController driverController;

  public AimAtGoal(SwerveSubsystem drivebase, CommandXboxController driverController) {
    this.drivebase = drivebase;
    this.driverController = driverController;

    addRequirements(drivebase);
  }

  @Override
  public void execute() {
    SwerveInputStream aimAt = SwerveInputStream.of(
        drivebase.getSwerveDrive(), 
        () -> driverController.getLeftX() * -1, 
        () -> driverController.getLeftY() * -1
    )
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(OperatorConstants.TRANSLATION_SCALE)
        .scaleRotation(-OperatorConstants.ROTATION_SCALE)
        .allianceRelativeControl(true)
        .aim(FieldConstants.centerPose);

    // 4. Send to drivebase
    drivebase.driveFieldOriented(aimAt);
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.driveFieldOriented(new ChassisSpeeds());
  }
}