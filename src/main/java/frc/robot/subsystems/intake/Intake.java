package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    
    @AutoLogOutput(key = "Intake/State")
    private IntakeState state = IntakeState.IDLE;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public Command intake() { return runOnce(() -> state = IntakeState.INTAKING); }
    public Command eject() { return runOnce(() -> state = IntakeState.EJECTING); }
    public Command stop() { return runOnce(() -> state = IntakeState.IDLE); }

    public IntakeState getState() {
        return state;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        switch (state) {
            case IDLE -> io.setRollerVoltage(0.0);
            case INTAKING -> io.setRollerVoltage(8.0);
            case EJECTING -> io.setRollerVoltage(-8.0);
        }
    }

    public enum IntakeState {
        IDLE,
        INTAKING, // Rollers spinning in
        EJECTING   // Rollers spinning out
    }
}
