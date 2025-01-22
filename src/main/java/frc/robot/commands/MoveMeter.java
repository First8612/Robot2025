package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MoveMeter extends Command {
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
    private CommandSwerveDrivetrain drivetrain;

    public MoveMeter(CommandSwerveDrivetrain drivetrain) {
        super();
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        drivetrain.applyRequest(() -> drive.withVelocityX(0.1));
    }
}
