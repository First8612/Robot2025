package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MoveMeters extends Command {
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private CommandSwerveDrivetrain drivetrain;
    private double targetX = 0;
    private double meters;

    public MoveMeters(double meters, CommandSwerveDrivetrain drivetrain) {
        super();
        this.meters = meters;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        targetX = drivetrain.getState().Pose.getX() + meters;
    }

    @Override
    public void execute() {
        System.out.println("Moving 1");
        drivetrain.setControl(forwardStraight.withVelocityX(0.5).withVelocityY(0));
    }

    @Override
    public boolean isFinished() {
        return drivetrain.getState().Pose.getX() > targetX;
    }
}
