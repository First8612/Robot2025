package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Ascender;

public class GoToPreset extends Command {
    private Ascender ascender;
    private int position;

    public GoToPreset(int position, Ascender ascender) {
        super();
        this.position = position;
        this.ascender = ascender;
    }

    @Override
    public void initialize() {
        ascender.goToPosition(position);
    }

    @Override
    public boolean isFinished() {
        return ascender.isAtPosition();
    }
}
