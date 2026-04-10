package frc.robot.subsystems.shooter;

import frc.robot.util.TunableControls.TunableControlConstants;
import frc.robot.util.TunableControls.TunablePIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public ShooterState getState() {
        return state;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        double error = Math.abs(inputs.flywheelVelocityRadPerSec - targetVelocity);
        boolean atSpeed = error < 4.0 && targetVelocity != 0;

        switch (state) {
            case IDLE -> {
                io.setFlywheelVoltage(0.0);
                io.setKickerVoltage(0.0);
                if (this.targetVelocity > 0.1) state = ShooterState.SPOOLING;
            }

            case SPOOLING -> {
                runFlywheel(targetVelocity); 
                if (atSpeed) state = ShooterState.SHOOTING;   
            }

            case SHOOTING -> {
                runFlywheel(targetVelocity);
                if (error < 8.0) {
                    io.setKickerVoltage(-12.0);
                } else {
                    io.setKickerVoltage(0.0); // Wait for recovery
                }
            }

            case EJECTING -> {
                io.setFlywheelVoltage(-4.0);
                io.setKickerVoltage(5.0);
            }
        }
    }

    public Command shoot(double velocity) {
        return runOnce(() -> {
            this.targetVelocity = velocity;
            this.state = ShooterState.SPOOLING;
        }).andThen(Commands.idle(this));
    }

    public Command stop() {
        return runOnce(() -> {
            this.targetVelocity = 0.0;
            this.state = ShooterState.IDLE;
        });
    }

    public Command eject() {
        return run(() -> this.state = ShooterState.EJECTING)
               .finallyDo(() -> this.state = ShooterState.IDLE);
    }

    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
        
        if (state == ShooterState.IDLE && velocity > 0.1) {
            state = ShooterState.SPOOLING;
        }
    }

    private void runFlywheel(double velocity) {
        double ff = controller.getParams().getSimpleFeedforward().calculate(velocity);
        double pid = controller.calculate(inputs.flywheelVelocityRadPerSec, velocity);
        io.setFlywheelVoltage(ff + pid);
    }

    public enum ShooterState {
        IDLE,      // Off
        SPOOLING,  // Getting up to speed
        SHOOTING,   // Firing
        EJECTING   // Clearing a jam
    }
}
