package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.StabbyThingy;

public class ForkEjectCoral extends ParallelDeadlineGroup {
    public ForkEjectCoral(StabbyThingy fork) {
        super(
            new SequentialCommandGroup(
                new ForkWaitForCoralPresence(false, fork),
                new WaitCommand(0.5)),
            new RunCommand(() -> fork.feed(true), fork)
        );
    }
}
