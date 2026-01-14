package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import frc.robot.SwerveConstants.ModuleConstants;

public final class Configs {
    public static final class ThriftySwerve {
        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
        public static final SparkMaxConfig steerConfig = new SparkMaxConfig();

        static {
            // --- Drive Motor Configuration ---
            driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ModuleConstants.kDriveCurrentLimit);

            driveConfig.encoder
                .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
                .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

            driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Example PID: adjust kV and kP based on your robot's weight/speed
                .pid(0.1, 0, 0) 
                .feedForward
                    .kV(1.0 / 4.5); // 1.0 / Max Velocity m/s

            // --- Steer Motor Configuration ---
            steerConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ModuleConstants.kSteerCurrentLimit);

            steerConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(1.0, 0, 0) // Starting P gain for steering
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI);
        }
    }
}