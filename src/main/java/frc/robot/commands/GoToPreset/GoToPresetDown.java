package frc.robot.commands.GoToPreset;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Ascender;

public class GoToPresetDown extends SequentialCommandGroup {
    public GoToPresetDown(int position, Ascender ascender) {
        super(
            new GoToPresetPivot(position, ascender),
            new GoToPresetWrist(position, ascender),
            new GoToPresetAscend(position, ascender)
        );
    }
}
