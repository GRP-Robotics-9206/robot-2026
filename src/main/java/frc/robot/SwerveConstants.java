package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * SwerveConstants stores all hardware IDs, physical dimensions, and conversion factors.
 * Grouped into inner classes to maintain a "DriveConstants.kConstant" naming convention.
 */
public class SwerveConstants {

    public static final class ModuleConstants {
        // --- Physical Wheel Specs ---
        public static final double kWheelDiameterInches = 4.0;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelDiameterInches);
        
        // --- Gear Ratios ---
        // Format: (Driven Teeth / Driving Teeth). 
        // Example: L2 SDS is 6.75:1. This means the motor spins 6.75 times for every 1 wheel rotation.
        public static final double kDriveGearRatio = 1.0 / 6.75;
        public static final double kSteerGearRatio = 1.0 / 25.0;

        // --- Conversion Factors ---
        // Multiplied by motor rotations to get meters traveled: (Rotations * GearRatio * Circumference)
        public static final double kDriveEncoderRot2Meter = kDriveGearRatio * Math.PI * kWheelDiameterMeters;
        
        // Multiplied by RPM to get meters per second: (EncoderRot2Meter / 60 seconds)
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        
        // Multiplied by motor rotations to get radians: (Rotations * GearRatio * 2PI Radians)
        public static final double kSteerEncoderRot2Rad = kSteerGearRatio * 2 * Math.PI;
        
        // Multiplied by RPM to get radians per second
        public static final double kSteerEncoderRPM2RadPerSec = kSteerEncoderRot2Rad / 60.0;

        // --- Hardware Limits & Safety ---
        public static final int kDriveCurrentLimit = 40; // Prevents tripping breakers during hard acceleration
        public static final int kSteerCurrentLimit = 20; // Steer motors don't need as much torque
        public static final double kNominalVoltage = 12.0; // Used for Voltage Compensation
    }

    public static final class DriveConstants {
        // Distance between center of left and right wheels
        public static final double kTrackWidth = Units.inchesToMeters(25.0);
        // Distance between center of front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(25.0);

        /**
         * Kinematics maps desired chassis speed (VX, VY, Omega) to individual module states.
         * WPILib Coordinates: +X is Forward, +Y is Left.
         */
        
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),   // Front Left (+X, +Y)
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // Front Right (+X, -Y)
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  // Back Left (-X, +Y)
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)  // Back Right (-X, -Y)
        );
    }

    /**
     * Module-specific constants. 
     * Angle Offset: The absolute encoder reading (in radians) when the wheel is facing perfectly forward.
     */
    public static final class SwerveModules {
        
        public static final class FrontRight {
            public static final int kDriveMotorID = 4;
            public static final int kSteerMotorID = 5;
            public static final int kAbsoluteEncoderID = 2;
            public static final double kAngleOffsetRad = Units.degreesToRadians(275.0);
            
            public static final boolean kDriveMotorInverted = false;
            public static final boolean kSteerMotorInverted = false;
            public static final boolean kAbsoluteEncoderInverted = false;
        }

        public static final class FrontLeft {
            public static final int kDriveMotorID = 6;
            public static final int kSteerMotorID = 7;
            public static final int kAbsoluteEncoderID = 3;
            public static final double kAngleOffsetRad = Units.degreesToRadians(257.0);
            
            public static final boolean kDriveMotorInverted = false;
            public static final boolean kSteerMotorInverted = false;
            public static final boolean kAbsoluteEncoderInverted = false;
        }

        public static final class BackLeft {
            public static final int kDriveMotorID = 8;
            public static final int kSteerMotorID = 9;
            public static final int kAbsoluteEncoderID = 1;
            public static final double kAngleOffsetRad = Units.degreesToRadians(157.0);
            
            public static final boolean kDriveMotorInverted = false;
            public static final boolean kSteerMotorInverted = false;
            public static final boolean kAbsoluteEncoderInverted = false;
        }

        public static final class BackRight {
            public static final int kDriveMotorID = 10;
            public static final int kSteerMotorID = 11;
            public static final int kAbsoluteEncoderID = 0;
            public static final double kAngleOffsetRad = Units.degreesToRadians(277.0);
            
            public static final boolean kDriveMotorInverted = false;
            public static final boolean kSteerMotorInverted = false;
            public static final boolean kAbsoluteEncoderInverted = false;
        }
    }
}