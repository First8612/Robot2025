package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Ascender;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StabbyThingy;

public class ApproachAndScoreAtPosition extends SequentialCommandGroup {
    public ApproachAndScoreAtPosition(
        int position,
        CommandSwerveDrivetrain drivetrain,
        Ascender ascender,
        StabbyThingy fork,
        Limelight limelight
    ) {
        super();

        addCommands(

            ascender.goToPosition(1), // go to station
            Commands.runOnce(() -> fork.inFork(0.25, false), fork),
            ascender.goToPosition(0), // grab the coral,
            new ForkWaitForCoralPresence(true, fork),
            ascender.goToPosition(position),

            // align and approach
            new AlignToTag(limelight, drivetrain),
            new MoveMeters(0.1, drivetrain),
            ascender.goToPosition(1), // go to station to score
            new MoveMeters(-0.25, drivetrain)
        );
    }
}
