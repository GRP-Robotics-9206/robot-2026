package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
    // Basic Dimensions
    public static final Distance FIELD_LENGTH = Inches.of(650.12);
    public static final Distance FIELD_WIDTH = Inches.of(316.64);
    public static final Distance ALLIANCE_ZONE = Inches.of(156.06);

    public static class Hub {
        public static final Distance X_OFFSET = Inches.of(181.56);
        public static final Distance TARGET_HEIGHT = Inches.of(56.4);
        
        public static final Translation3d BLUE_HUB = 
            new Translation3d(X_OFFSET, FIELD_WIDTH.div(2), TARGET_HEIGHT);
        public static final Translation3d RED_HUB = 
            new Translation3d(FIELD_LENGTH.minus(X_OFFSET), FIELD_WIDTH.div(2), TARGET_HEIGHT);

        public static Translation3d getTarget() {
            return isRed() ? RED_HUB : BLUE_HUB;
        }
    }

    /** * Trench and Bump zones for both sides of the field.
     * Useful for PathPlanner obstacle avoidance.
     */
    public static class Obstacles {
        public static final Distance LENGTH_X = Inches.of(47);
        public static final Distance TRENCH_WIDTH_Y = Inches.of(49.86);
        public static final Distance BUMP_WIDTH_Y = Inches.of(73);

        public static final Distance BLUE_X = Inches.of(181.56);
        public static final Translation2d BLUE_TRENCH_CENTER = new Translation2d(BLUE_X, TRENCH_WIDTH_Y.div(2));
        
        public static final Distance RED_X = FIELD_LENGTH.minus(BLUE_X);
        public static final Translation2d RED_TRENCH_CENTER = new Translation2d(RED_X, TRENCH_WIDTH_Y.div(2));

        public static Distance getActiveX() {
            return isRed() ? RED_X : BLUE_X;
        }
    }

    
    /** Tower / Rung reference points */
    public static class Tower {
        public static final Distance X = Inches.of(49.25);
        public static final Distance CENTER_X = Inches.of(18);
        public static final Distance CENTER_Y = FIELD_WIDTH.div(2).minus(Inches.of(11.46));
        public static final Distance WIDTH = Inches.of(47);
    }

    /** Depot / Loading Zone reference points */
    public static class Depot {
        public static final Distance WIDTH = Inches.of(42.0);
        public static final Distance DEPTH = Inches.of(27.0);
        public static final Distance HEIGHT = Inches.of(1.125);
    }

    public static boolean isRed() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }
}