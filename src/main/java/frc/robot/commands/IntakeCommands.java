package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
    public static Command setup(Intake intake) {
        return intake.deploy();
    }

    /**
     *  Starts the rollers.
     */
    public static Command intake(Intake intake) {
        return intake.intake()
        .onlyIf(intake::deployed)
        .finallyDo(intake::idle);
    }

    /**
     * A safe stow command that stops rollers before pulling the pivot back.
     */
    public static Command safeStow(Intake intake) {
        return Commands.sequence(
            intake.stop(), // Stops rollers
            intake.stow()  // Pulls pivot up
        );
    }
}
