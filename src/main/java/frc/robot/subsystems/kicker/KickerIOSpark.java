package frc.robot.subsystems.kicker;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.subsystems.kicker.KickerConstants.*;

public class KickerIOSpark implements KickerIO {
    private final SparkMax motor;

    public KickerIOSpark() {
        this.motor = new SparkMax(kickerCanID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(30);
        
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.velocityRadPerSec = motor.getEncoder().getVelocity() * 0.1047;
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}