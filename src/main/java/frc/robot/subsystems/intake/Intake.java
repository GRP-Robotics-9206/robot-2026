package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private IntakeState state = IntakeState.STOWED;

    private final Debouncer stallDebouncer = new Debouncer(0.1);

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public Command deploy() {
        return this.setGoalState(IntakeState.DEPLOYING).withName("Deploy");
    }
    
    public Command stow() {
        return this.setGoalState(IntakeState.STOWED).withName("Stow");
    }

    public Command intake() {
        return this.setGoalState(IntakeState.INTAKING).withName("Intake");
    }

    public Command eject() {
        return this.setGoalState(IntakeState.EJECTING).withName("Eject");
    }

    public Command stop() {
        return this.setGoalState(IntakeState.DEPLOYED).withName("Stop");
    }

    public Command setGoalState(IntakeState newState) {
        return runOnce(() -> this.state = newState);
    }

    public Boolean deployed() {
        return state != IntakeState.STOWED && state != IntakeState.DEPLOYING;
    }

    public IntakeState getState() {
        return state;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        switch (state) {
            case STOWED -> {
                io.setPivotVoltage(-1.0); 
                io.setRollerVoltage(0.0);
            }

            case DEPLOYING -> {
                io.setPivotVoltage(2.0); 
                
                if (inputs.pivotPositionRad > 0.5 || stallDebouncer.calculate(inputs.pivotVelocityRadPerSec > 0.1)) {
                    state = IntakeState.DEPLOYED;
                }
            }

            case DEPLOYED -> {
                io.setPivotVoltage(0.0);
                io.setRollerVoltage(0.0);
            }

            case INTAKING -> {
                io.setPivotVoltage(0.0);
                io.setRollerVoltage(8.0);
            }

            case EJECTING -> {
                io.setPivotVoltage(0.0);
                io.setRollerVoltage(-8.0);
            }
        }
    }

    public enum IntakeState {
        STOWED,    // Pivot up, rollers off
        DEPLOYED,  // Pivot down, rollers off
        DEPLOYING, // Pivoting down, rollers off
        INTAKING,  // Pivot down, rollers spinning in
        EJECTING   // Pivot down, rollers spinning out
    }
}
