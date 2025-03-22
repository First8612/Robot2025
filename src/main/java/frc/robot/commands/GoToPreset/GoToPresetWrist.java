package frc.robot.commands.GoToPreset;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Ascender;

public class GoToPresetWrist extends Command {
    private Ascender ascender;
    private int position;

    public GoToPresetWrist(int position, Ascender ascender) {
        super();
        this.position = position;
        this.ascender = ascender;
    }

    @Override
    public void initialize() {
        ascender.goToPositionWrist(position);
    }

    @Override
    public boolean isFinished() {
        return ascender.isWristAtPosition();
    }
}
