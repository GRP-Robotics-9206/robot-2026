package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double rollerVelocityRadPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
  public default void setRollerVoltage(double volts) {}
}