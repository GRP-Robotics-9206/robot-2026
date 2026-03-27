package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
    
    @AutoLogOutput(key = "Kicker/State")
    private KickerState state = KickerState.IDLE;

    public Kicker(KickerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Kicker", inputs);

        switch (state) {
            case IDLE -> {
                io.setVoltage(0.0);
            }

            case KICKING -> {
                io.setVoltage(-5.0);
            }

            case EJECTING -> {
                io.setVoltage(5.0);
            }
        }

    }

    public Command run() {
        return setGoalState(KickerState.KICKING).withName("Kicking");
    }

    public Command eject() {
        return setGoalState(KickerState.EJECTING).withName("Ejecting");
    }

    public Command stop() {
        return setGoalState(KickerState.IDLE).withName("Stop");
    }

    public Command setGoalState(KickerState newState) {
        return runOnce(() -> this.state = newState);
    }

    public KickerState getState() {
        return state;
    }

    public enum KickerState {
        IDLE,      // Off
        KICKING,   // Kicking
        EJECTING   // Backfeeding balls
    }
}