package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakeIOSpark implements IntakeIO {
    private SparkBase rollerSpark;
    private SparkBase pivotSpark;
    private RelativeEncoder rollorEncoder;
    private RelativeEncoder pivotEncoder;

    public IntakeIOSpark() {
        rollerSpark = new SparkMax(rollerCanID, MotorType.kBrushless);
        pivotSpark = new SparkMax(rollerCanID, MotorType.kBrushless);

        pivotEncoder = pivotSpark.getEncoder();
        rollorEncoder = rollerSpark.getEncoder();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerVelocityRadPerSec = Units.rotationsToRadians(rollorEncoder.getVelocity() / 60.0);
        inputs.pivotPositionRad = Units.rotationsToRadians(pivotEncoder.getPosition());
        inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotEncoder.getVelocity() / 60.0);
        inputs.rollerAppliedVolts = rollerSpark.getAppliedOutput() * rollerSpark.getBusVoltage();
    }

    @Override
    public void setRollerVoltage(double volts) {
        rollerSpark.setVoltage(volts);
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotSpark.setVoltage(volts);
    }
}
