package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CustomMoveMeters extends Command {
    private final SwerveRequest.FieldCentric forwardStraight = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private CommandSwerveDrivetrain drivetrain;
    private double meters;
    private PIDController driveController = new PIDController(2, 0.1, 0);
    private SlewRateLimiter rateLimiter = new SlewRateLimiter(5);
    private int reverse = 1;
    private boolean doWeMoveForwards = true;

    public CustomMoveMeters(double meters, CommandSwerveDrivetrain drivetrain, boolean doWeMoveForwards) {
        super();
        this.meters = meters;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        driveController.setIZone(0.5);
        SmartDashboard.putData("MoveMeterDriveController", driveController);
    }

    @Override
    public void initialize() {
        DriverStation.getAlliance().ifPresent((x) -> {
            if (x == Alliance.Red)
            {
                reverse = -1;
            }
        });

        if(doWeMoveForwards == true){

            driveController.setSetpoint(
            drivetrain.getState().Pose.getX() + meters * reverse
        );
        rateLimiter.reset(0);

        }else if(doWeMoveForwards != true){
        driveController.setSetpoint(
            drivetrain.getState().Pose.getY() + meters * reverse
        );
        rateLimiter.reset(0);
        }
    }

    @Override
    public void execute() {
        //System.out.println("Moving 1");
        if(doWeMoveForwards == true){
            var velocity = driveController.calculate(drivetrain.getState().Pose.getX());
            velocity = rateLimiter.calculate(velocity);

            SmartDashboard.putNumber("MoveMeterDriveController.velocityX", velocity);

            drivetrain.setControl(forwardStraight.withVelocityX(velocity).withVelocityY(0));
        }else if(doWeMoveForwards != true){
            var velocity = driveController.calculate(drivetrain.getState().Pose.getY());
            velocity = rateLimiter.calculate(velocity);

            SmartDashboard.putNumber("MoveMeterDriveController.velocityY", velocity);

            drivetrain.setControl(forwardStraight.withVelocityX(velocity).withVelocityX(0));
        }
    }

    @Override
    public boolean isFinished() {
        //System.out.println(Math.abs(driveController.getError()) < 0.02);
        return Math.abs(driveController.getError()) < 0.02;
    }
}





