package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double flywheelVelocityRadPerSec = 0.0;
        public double flywheelAppliedVolts = 0.0;
        public double flywheelCurrentAmps = 0.0;
        public double kickerVelocityRadPerSec = 0.0;
        public double kickerAppliedVolts = 0.0;
        public double kickerCurrentAmps = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}
    public default void setFlywheelVoltage(double volts) {}
    public default void setKickerVoltage(double volts) {}
}
