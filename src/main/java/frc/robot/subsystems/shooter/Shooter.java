package frc.robot.subsystems.shooter;

import frc.robot.util.TunableControls.TunableControlConstants;
import frc.robot.util.TunableControls.TunablePIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private ShooterState state = ShooterState.IDLE;
    private final TunablePIDController controller;
    private double targetVelocity = 0.0;

    public Shooter(ShooterIO io) {
        this.io = io;

        this.controller = new TunablePIDController(
            new TunableControlConstants("Shooter", constants)
        );
    }

    public Command stop() {
        return setGoalState(ShooterState.IDLE);
    }

    public Command shoot(double velocity) {
        this.targetVelocity = velocity;
        return setGoalState(ShooterState.SPOOLING);
    }

    public Command setGoalState(ShooterState newState) {
        return runOnce(() -> this.state = newState);
    }

    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
        this.state = ShooterState.SPOOLING; 
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        switch (state) {
            case IDLE -> {
                io.setVoltage(0.0);
            }

            case SPOOLING -> {
                runFlywheel(targetVelocity);
                
                if (Math.abs(inputs.velocityRadPerSec - targetVelocity) < 2.0) {
                    state = ShooterState.READY;
                }
            }

            case READY -> {
                runFlywheel(targetVelocity);
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
