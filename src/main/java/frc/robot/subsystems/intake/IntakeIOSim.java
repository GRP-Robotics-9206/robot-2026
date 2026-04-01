package frc.robot.subsystems.intake;

public class IntakeIOSim implements IntakeIO {
    private double rollerVolts = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerAppliedVolts = rollerVolts;
        inputs.rollerVelocityRadPerSec = rollerVolts * 10.0; 
    }

    @Override
    public void setRollerVoltage(double volts) {
        this.rollerVolts = volts;
    }
}
