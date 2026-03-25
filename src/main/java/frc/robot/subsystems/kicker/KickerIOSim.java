package frc.robot.subsystems.kicker;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class KickerIOSim implements KickerIO {
    private final SlewRateLimiter voltageFilter = new SlewRateLimiter(40.0);
    
    private double appliedVolts = 0.0;
    private double currentVelocityRadPerSec = 0.0;

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        double filteredVolts = voltageFilter.calculate(appliedVolts);
        currentVelocityRadPerSec = (filteredVolts / 12.0) * 500.0;

        inputs.appliedVolts = filteredVolts;
        inputs.velocityRadPerSec = currentVelocityRadPerSec;
        inputs.currentAmps = Math.abs(filteredVolts) * 2.0;
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
    }
}