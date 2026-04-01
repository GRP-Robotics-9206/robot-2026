package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterIOSim implements ShooterIO {
    private static final DCMotor flywheelGearbox = DCMotor.getNEO(1);
    private static final double flywheelReduction = 1.0 / gearRatio;
    private static final double flywheelMomentOfInertia = 0.005;

    private static final DCMotor kickerGearbox = DCMotor.getNeo550(1);
    private static final double kickerReduction = 1.0; 
    private static final double kickerMomentOfInertia = 0.0005;

    private final FlywheelSim flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(flywheelGearbox, flywheelMomentOfInertia, flywheelReduction),
        flywheelGearbox
    );

    private final FlywheelSim kickerSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(kickerGearbox, kickerMomentOfInertia, kickerReduction),
        kickerGearbox
    );

    private double flywheelAppliedVolts = 0.0;
    private double kickerAppliedVolts = 0.0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        flywheelSim.update(0.020); 
        kickerSim.update(0.020);

        inputs.flywheelVelocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
        inputs.flywheelAppliedVolts = flywheelAppliedVolts;
        inputs.flywheelCurrentAmps = flywheelSim.getCurrentDrawAmps();

        inputs.kickerVelocityRadPerSec = kickerSim.getAngularVelocityRadPerSec();
        inputs.kickerAppliedVolts = kickerAppliedVolts;
        inputs.kickerCurrentAmps = kickerSim.getCurrentDrawAmps();
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        flywheelAppliedVolts = volts;
        flywheelSim.setInputVoltage(volts);
    }

    @Override
    public void setKickerVoltage(double volts) {
        kickerAppliedVolts = volts;
        kickerSim.setInputVoltage(volts);
    }
}