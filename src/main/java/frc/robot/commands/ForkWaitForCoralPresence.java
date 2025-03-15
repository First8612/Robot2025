package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StabbyThingy;

public class ForkWaitForCoralPresence extends Command {
    private final StabbyThingy stabbyThingy;
    private boolean present;

    public ForkWaitForCoralPresence(boolean present, StabbyThingy stabbyThingy) {
        this.present = present;
        this.stabbyThingy = stabbyThingy;
        addRequirements(stabbyThingy);
    }

    @Override
    public boolean isFinished() {
        return stabbyThingy.getCoralPresent() == present;
    }
}
