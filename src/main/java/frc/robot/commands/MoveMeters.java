package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MoveMeters extends Command {
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private CommandSwerveDrivetrain drivetrain;
    private double meters;
    private PIDController driveController = new PIDController(2, 0.1, 0);
    private SlewRateLimiter rateLimiter = new SlewRateLimiter(5);

    public MoveMeters(double meters, CommandSwerveDrivetrain drivetrain) {
        super();
        this.meters = meters;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        driveController.setIZone(0.5);
        SmartDashboard.putData("MoveMeterDriveController", driveController);
    }

    @Override
    public void initialize() {
        driveController.setSetpoint(
            drivetrain.getState().Pose.getX() + meters
        );
        rateLimiter.reset(0);
    }

    @Override
    public void execute() {
        System.out.println("Moving 1");

        var velocity = driveController.calculate(drivetrain.getState().Pose.getX());
        velocity = rateLimiter.calculate(velocity);

        SmartDashboard.putNumber("MoveMeterDriveController.velocity", velocity);

        drivetrain.setControl(forwardStraight.withVelocityX(velocity).withVelocityY(0));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(driveController.getError()) < 0.02;
    }
}
