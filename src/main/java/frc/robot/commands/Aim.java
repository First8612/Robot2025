package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Aim extends Command {
    private final PIDController pidController = new PIDController(0.07, 0, 0);
    private CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public Aim(CommandSwerveDrivetrain drivetrain) {
        super();
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        var tx = LimelightHelpers.getTX("limelight");

        var rotation = pidController.calculate(tx);
        System.out.println(rotation);

        drivetrain.setControl(drive.withVelocityX(0.5)
            .withVelocityY(0)
            .withRotationalRate(rotation));
    }
}
