package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoToPreset.GoToPresetPivot;
import frc.robot.subsystems.Ascender;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StabbyThingy;

public class DriveUpAndPlaceOn extends SequentialCommandGroup {
    public DriveUpAndPlaceOn(int position, boolean left, Ascender ascender, StabbyThingy fork, CommandSwerveDrivetrain drivetrain) {
        super(
            new ParallelCommandGroup(
                new GoToPresetPivot(position, ascender),
                new IWannaDumpSomeCoral(drivetrain, left)
            ),
            new ForkEjectCoral(fork),
            new MoveMeters(-0.5, drivetrain),
            new GoToPresetPivot(0, ascender)
        );
    }


}
