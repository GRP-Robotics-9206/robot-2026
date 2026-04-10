package frc.robot.subsystems.shooter;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterIOSpark implements ShooterIO {
    private SparkBase flywheelSpark;
    private SparkBase kickerSpark;

    public ShooterIOSpark() {
        SparkMaxConfig fwConfig = new SparkMaxConfig();
        fwConfig.encoder.velocityConversionFactor(motorRotationsRadPerSec * gearRatio);
        fwConfig.smartCurrentLimit(50);
        fwConfig.inverted(true);

        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        kickerConfig.smartCurrentLimit(30);

        flywheelSpark = new SparkMax(flywheelCanID, MotorType.kBrushless);
        flywheelSpark.configure(fwConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        kickerSpark = new SparkMax(kickerCanID, MotorType.kBrushless);
        kickerSpark.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelVelocityRadPerSec = flywheelSpark.getEncoder().getVelocity();
        inputs.flywheelAppliedVolts = flywheelSpark.getAppliedOutput() * flywheelSpark.getBusVoltage();
        inputs.flywheelCurrentAmps = flywheelSpark.getOutputCurrent();

        inputs.kickerVelocityRadPerSec = kickerSpark.getEncoder().getVelocity() * 0.1047;
        inputs.kickerAppliedVolts = kickerSpark.getAppliedOutput() * kickerSpark.getBusVoltage();
        inputs.kickerCurrentAmps = kickerSpark.getOutputCurrent();
    }

    @Override
    public void setFlywheelVoltage(double volts) { flywheelSpark.setVoltage(volts); }

    @Override
    public void setKickerVoltage(double volts) { kickerSpark.setVoltage(volts); }
}