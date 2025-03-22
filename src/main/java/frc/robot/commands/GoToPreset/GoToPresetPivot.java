package frc.robot.commands.GoToPreset;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Ascender;

public class GoToPresetPivot extends Command {
    private Ascender ascender;
    private int position;

    public GoToPresetPivot(int position, Ascender ascender) {
        super();
        this.position = position;
        this.ascender = ascender;
    }

    @Override
    public void initialize() {
        ascender.goToPositionPivot(position);
    }

    @Override
    public boolean isFinished() {
        return ascender.isPivotAtPosition();
    }
}
