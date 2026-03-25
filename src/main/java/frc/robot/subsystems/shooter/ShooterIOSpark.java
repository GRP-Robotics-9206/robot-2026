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

    public ShooterIOSpark() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.encoder.velocityConversionFactor(0.104719755);
        config.smartCurrentLimit(50);

        flywheelSpark = new SparkMax(flywheelCanID, MotorType.kBrushless);
        flywheelSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.velocityRadPerSec = flywheelSpark.getEncoder().getVelocity();
        inputs.appliedVolts = flywheelSpark.getAppliedOutput() * flywheelSpark.getBusVoltage();
        inputs.currentAmps = new double[] { flywheelSpark.getOutputCurrent() };
    }

    @Override
    public void setVoltage(double volts) {
        flywheelSpark.setVoltage(volts);
    }
}
