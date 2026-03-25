package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
    @AutoLog
    public static class KickerIOInputs {
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double velocityRadPerSec = 0.0;
    }

    public default void updateInputs(KickerIOInputs inputs) {}
    public default void setVoltage(double volts) {}
}
