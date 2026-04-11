package frc.robot.subsystems.shooter;

import frc.robot.util.TunableControls.ControlConstants;

public class ShooterConstants {
    public static final int flywheelCanID = 13;
    public static final int kickerCanID = 12;
    public static final double motorRotationsRadPerSec = (2.0 * Math.PI) / 60.0;
    public static final double gearRatio = 1.363;

    public static final ControlConstants constants = new ControlConstants()
        .withPID(0.074, 0.0, 0.0016)
        .withFeedforward(0.0154, 0.0)
        .withPhysical(0.2, 0.0)
        .withTolerance(4.0);   
}
