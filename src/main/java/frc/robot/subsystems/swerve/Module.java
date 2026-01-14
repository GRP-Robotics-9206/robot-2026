package frc.robot.subsystems.swerve;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Configs;

public class Module {
    private final SparkMax m_driveMotor;
    private final SparkMax m_steerMotor;

    private final RelativeEncoder m_driveEncoder;
    private final SparkClosedLoopController m_drivePID;
    private final SparkClosedLoopController m_steerPID;

    private final AnalogInput m_steerEncoder;
    private final double m_chassisAngularOffset; // The 'zero' position
    private final boolean m_encoderInverted;

    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public Module(int driveID, int steerID, int analogPort, double chassisAngularOffset, boolean driveInv, boolean steerInv, boolean encoderInv) {
        
        m_chassisAngularOffset = chassisAngularOffset;
        m_encoderInverted = encoderInv;

        m_driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        m_steerMotor = new SparkMax(steerID, MotorType.kBrushless);
        m_steerEncoder = new AnalogInput(analogPort);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_drivePID = m_driveMotor.getClosedLoopController();
        m_steerPID = m_steerMotor.getClosedLoopController();

        // Apply Drive Config (from Configs.java)
        SparkMaxConfig driveConfig = Configs.ThriftySwerve.driveConfig;
        driveConfig.inverted(driveInv);
        m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Apply Steer Config (from Configs.java)
        SparkMaxConfig steerConfig = Configs.ThriftySwerve.steerConfig;
        steerConfig.inverted(steerInv);
        m_steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_desiredState.angle = new Rotation2d(getRawEncoderRadians());

        m_driveEncoder.setPosition(0);
    }

    /**
     * Helper to get raw encoder radians from the RIO Analog Port.
     */
    private double getRawEncoderRadians() {
        double voltageRatio = m_steerEncoder.getVoltage() / RobotController.getVoltage5V();
        double angle = voltageRatio * 2.0 * Math.PI;
        return m_encoderInverted ? (2.0 * Math.PI - angle) : angle;
    }

    /**
     * Returns the current state of the module (Speed and Angle).
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_driveEncoder.getVelocity(),
            new Rotation2d(getRawEncoderRadians() - m_chassisAngularOffset)
        );
    }

    /**
     * Returns the current position of the module (Distance and Angle).
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(),
            new Rotation2d(getRawEncoderRadians() - m_chassisAngularOffset)
        );
    }

    /**
     * Sets the desired state for the module.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // 1. Apply chassis angular offset to the desired state (matching EasySwerve logic)
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // 2. Optimize the state based on CURRENT raw encoder position
        correctedDesiredState.optimize(new Rotation2d(getRawEncoderRadians()));

        // 3. Command Drive Motor (Onboard SparkMax Velocity PID)
        m_drivePID.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);

        // 4. Command Steer Motor (Onboard SparkMax Position PID via Remote Feedback)
        // We pass the current RIO encoder reading as the 'actual' position for the Spark PID
        m_steerPID.setSetpoint(correctedDesiredState.angle.getRadians(),ControlType.kPosition);

        m_desiredState = desiredState;
    }

    public void resetEncoders() {
        m_driveEncoder.setPosition(0);
    }
}