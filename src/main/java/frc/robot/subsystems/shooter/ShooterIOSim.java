package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterIOSim implements ShooterIO {
    private static final DCMotor m_gearbox = DCMotor.getNEO(1);
    private static final double m_reduction = 1.0 / gearRatio;
    private static final double m_jKgMetersSquared = 0.005;

    private final FlywheelSim sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(m_gearbox, m_jKgMetersSquared, m_reduction),
        m_gearbox
    );

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Standard 20ms loop update
        sim.update(0.020); 

        // Update the AdvantageKit inputs
        inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = new double[] { sim.getCurrentDrawAmps() };
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
        sim.setInputVoltage(volts);
    }
}
