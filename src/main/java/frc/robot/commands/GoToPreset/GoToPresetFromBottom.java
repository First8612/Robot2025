package frc.robot.commands.GoToPreset;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Ascender;

public class GoToPresetFromBottom extends SequentialCommandGroup {
    public GoToPresetFromBottom(int position, Ascender ascender) {
        super(
            new GoToPresetAscend(position, ascender),
            new GoToPresetPivot(position, ascender),
            new GoToPresetWrist(position, ascender)
        );
    }
}
