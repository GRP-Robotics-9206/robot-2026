package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class IntakeIOSim implements IntakeIO {
    private final ProfiledPIDController profile = 
        new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(10, 20));

    private double currentPosRad = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        currentPosRad = profile.calculate(currentPosRad);
        
        inputs.pivotPositionRad = currentPosRad;
        inputs.pivotVelocityRadPerSec = profile.getSetpoint().velocity;
    }

    @Override
    public void setPivotVoltage(double volts) {
        if (volts > 0.1) profile.setGoal(1.5);
        if (volts < -0.1) profile.setGoal(0.0);
    }
}
