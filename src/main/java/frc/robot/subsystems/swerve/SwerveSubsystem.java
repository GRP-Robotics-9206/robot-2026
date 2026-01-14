package frc.robot.subsystems.swerve;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.SwerveConstants.SwerveModules;

public class SwerveSubsystem extends SubsystemBase {
  private final Module m_frontLeft = new Module(
      SwerveModules.FrontLeft.kDriveMotorID,
      SwerveModules.FrontLeft.kSteerMotorID,
      SwerveModules.FrontLeft.kAbsoluteEncoderID,
      SwerveModules.FrontLeft.kAngleOffsetRad,
      SwerveModules.FrontLeft.kDriveMotorInverted,
      SwerveModules.FrontLeft.kSteerMotorInverted,
      SwerveModules.FrontLeft.kAbsoluteEncoderInverted);

  private final Module m_frontRight = new Module(
      SwerveModules.FrontRight.kDriveMotorID,
      SwerveModules.FrontRight.kSteerMotorID,
      SwerveModules.FrontRight.kAbsoluteEncoderID,
      SwerveModules.FrontRight.kAngleOffsetRad,
      SwerveModules.FrontRight.kDriveMotorInverted,
      SwerveModules.FrontRight.kSteerMotorInverted,
      SwerveModules.FrontRight.kAbsoluteEncoderInverted);

  private final Module m_rearLeft = new Module(
      SwerveModules.BackLeft.kDriveMotorID,
      SwerveModules.BackLeft.kSteerMotorID,
      SwerveModules.BackLeft.kAbsoluteEncoderID,
      SwerveModules.BackLeft.kAngleOffsetRad,
      SwerveModules.BackLeft.kDriveMotorInverted,
      SwerveModules.BackLeft.kSteerMotorInverted,
      SwerveModules.BackLeft.kAbsoluteEncoderInverted);

  private final Module m_rearRight = new Module(
      SwerveModules.BackRight.kDriveMotorID,
      SwerveModules.BackRight.kSteerMotorID,
      SwerveModules.BackRight.kAbsoluteEncoderID,
      SwerveModules.BackRight.kAngleOffsetRad,
      SwerveModules.BackRight.kDriveMotorInverted,
      SwerveModules.BackRight.kSteerMotorInverted,
      SwerveModules.BackRight.kAbsoluteEncoderInverted);

  private AHRS m_gyro;

  private final SwerveDriveOdometry m_odometry;

  public SwerveSubsystem() {
    try {
        m_gyro = new AHRS(NavXComType.kUSB1);
    } catch (UnsatisfiedLinkError | Exception e) {
        System.out.println("NavX not found or Simulation detected: " + e.getMessage());
        m_gyro = null; 
    }

    m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        getRotation2d(),
        getModulePositions());
  }

  @Override
  public void periodic() {
    m_odometry.update(getRotation2d(), getModulePositions());
  }

  /** Helper to get Module Positions array for Odometry updates */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  public Rotation2d getRotation2d() {
    // If gyro exists and is connected, use it. Otherwise return 0.
    if (m_gyro != null && m_gyro.isConnected()) {
        return Rotation2d.fromDegrees(m_gyro.getAngle());
    }
    // Return 0 during simulation so the code doesn't crash
    return new Rotation2d(0);
  }


  /** Returns the current states (velocity/angle) of all modules. */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  /** Returns the current velocity of the robot as a ChassisSpeeds object. */
  public ChassisSpeeds getRobotVelocity() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Max speeds usually come from your ModuleConstants or DriveConstants
    // Adjust these multipliers if the robot is too fast/slow
    double xSpeedDelivered = xSpeed * 4.5; // m/s
    double ySpeedDelivered = ySpeed * 4.5; // m/s
    double rotDelivered = rot * 2.0 * Math.PI; // rad/s

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
    );
            
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 4.5);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  // --- Utility Methods ---

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void stop() {
    drive(0, 0, 0, false);
  }

  public void zeroGyro() {
    if (m_gyro != null) {
        m_gyro.reset();
    }
  }
}