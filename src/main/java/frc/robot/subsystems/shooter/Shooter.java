package frc.robot.subsystems.shooter;

import frc.robot.util.TunableControls.TunableControlConstants;
import frc.robot.util.TunableControls.TunablePIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final TunablePIDController controller;

    @AutoLogOutput(key = "Shooter/State")
    private ShooterState state = ShooterState.IDLE;

    @AutoLogOutput(key = "Shooter/TargetVelocity")
    private double targetVelocity = 0.0;

    public Shooter(ShooterIO io) {
        this.io = io;

        this.controller = new TunablePIDController(
            new TunableControlConstants("Shooter", constants)
        );
    }

    public Command stop() {
        this.targetVelocity = 0.0;
        return setGoalState(ShooterState.IDLE).withName("Stop");
    }

    public Command shoot(double velocity) {
        this.targetVelocity = velocity;
        return setGoalState(ShooterState.SPOOLING).withName("Spooling");
    }

    public Command setGoalState(ShooterState newState) {
        return runOnce(() -> this.state = newState);
    }

    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
    }

    public Boolean atSetpoint() {
        return state == ShooterState.READY;
    }

    public ShooterState getState() {
        return state;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/IsReady", atSetpoint());

        double error = Math.abs(inputs.velocityRadPerSec - targetVelocity);

        switch (state) {
            case IDLE -> {
                io.setVoltage(0.0);
                if (this.targetVelocity > 0.1) state = ShooterState.SPOOLING;
            }

            case SPOOLING -> {
                runFlywheel(targetVelocity);
                
                if (error < 4.0) {
                    state = ShooterState.READY;
                }
            }

            case READY -> {
                runFlywheel(targetVelocity);

                if (Math.abs(inputs.velocityRadPerSec - targetVelocity) > 12.0) {
                    state = ShooterState.SPOOLING;
                }
            }

            case EJECTING -> {
                io.setVoltage(-4.0);
            }
        }
    }

    private void runFlywheel(double velocity) {
        double ff = controller.getParams().getSimpleFeedforward().calculate(velocity);
        double pid = controller.calculate(inputs.velocityRadPerSec, velocity);
        io.setVoltage(ff + pid);
    }

    public enum ShooterState {
        IDLE,      // Off
        SPOOLING,  // Getting up to speed
        READY,     // At speed, ready to fire
        EJECTING   // Clearing a jam
    }
}
