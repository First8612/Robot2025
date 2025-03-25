package frc.robot.commands.GoToPreset;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Ascender;

public class GoToPresetUp extends SequentialCommandGroup {
    public GoToPresetUp(int position, Ascender ascender) {
        super(
            new ParallelCommandGroup(
                new GoToPresetPivot(position, ascender),
                new GoToPresetAscend(position, ascender)
            ),
            new GoToPresetWrist(position, ascender)
        );
    }
}
