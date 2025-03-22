package frc.robot.commands.GoToPreset;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Ascender;

public class GoToPresetUp extends SequentialCommandGroup {
    public GoToPresetUp(int position, Ascender ascender) {
        super(
            new GoToPresetAscend(position, ascender),
            new GoToPresetWrist(position, ascender),
            new GoToPresetPivot(position, ascender)
        );
    }
}
