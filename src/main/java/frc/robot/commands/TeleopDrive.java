package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;// Ensure this points to your actual constants file
import java.util.function.DoubleSupplier;
import frc.robot.Constants.OperatorConstants;

public class TeleopDrive extends Command {
    private final SwerveSubsystem m_swerve;
    private final DoubleSupplier m_xSpeed, m_ySpeed, m_rot;

    public TeleopDrive(SwerveSubsystem swerve,DoubleSupplier xSpeed,DoubleSupplier ySpeed, DoubleSupplier rot) {
        this.m_swerve = swerve;
        this.m_xSpeed = xSpeed;
        this.m_ySpeed = ySpeed;
        this.m_rot = rot;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double x = -MathUtil.applyDeadband(m_xSpeed.getAsDouble(), OperatorConstants.DEADBAND)  * OperatorConstants.TRANSLATION_SCALE;         
        double y = -MathUtil.applyDeadband(m_ySpeed.getAsDouble(), OperatorConstants.DEADBAND)  * OperatorConstants.TRANSLATION_SCALE;   
        double r = -MathUtil.applyDeadband(m_rot.getAsDouble(), OperatorConstants.DEADBAND) * OperatorConstants.ROTATION_SCALE;

        m_swerve.drive(x, y, r, true);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }
}