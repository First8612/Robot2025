package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MoveMeter extends Command {
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private CommandSwerveDrivetrain drivetrain;

    public MoveMeter(CommandSwerveDrivetrain drivetrain) {
        super();
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);           
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        System.out.println("Moving 1");
        drivetrain.setControl(forwardStraight.withVelocityX(0.5).withVelocityY(0));
    }
}
