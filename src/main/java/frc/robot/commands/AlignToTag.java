package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToTag extends Command {
    private Limelight limelight;
    private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private CommandSwerveDrivetrain drivetrain;
    private PIDController xController = new PIDController(1.3, 0, 0);
    private PIDController yController = new PIDController(1.3, 0, 0);
    private PIDController rotationController = new PIDController(5, 0, 0);

    private boolean hadTag = false;

    public AlignToTag(Limelight limelight, CommandSwerveDrivetrain drivetrain) {
        super();
        this.limelight = limelight;
        this.drivetrain = drivetrain;

        SmartDashboard.putData("AlignToTag/xController", xController);
        SmartDashboard.putData("AlignToTag/yController", yController);
        SmartDashboard.putData("AlignToTag/rotationController", rotationController);
        SmartDashboard.putData(this);
    }

    @Override
    public void initialize() {
        xController.reset();
        rotationController.reset();
    }

    @Override
    public boolean isFinished() {
        if (hadTag && !limelight.hasTarget()) {
            return true;
        }
        
        return false;
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("AlignToTag/targetPose/exists", limelight.hasTarget());
        if (limelight.hasTarget())
            return;

        hadTag = true;

        var targetPose = limelight.getTargetPose3d_CameraSpace();
        SmartDashboard.putNumber("AlignToTag/targetPose/x", targetPose.getX());
        SmartDashboard.putNumber("AlignToTag/targetPose/y", targetPose.getY());
        SmartDashboard.putNumber("AlignToTag/targetPose/z", targetPose.getZ());
        SmartDashboard.putNumber("AlignToTag/targetPose/rot-x", targetPose.getRotation().getX());
        SmartDashboard.putNumber("AlignToTag/targetPose/rot-y", targetPose.getRotation().getY());
        SmartDashboard.putNumber("AlignToTag/targetPose/rot-z", targetPose.getRotation().getZ());

        var drive = robotDrive;
        drive.withRotationalRate(
            rotationController.calculate(targetPose.getX())
        );

        drive.withVelocityY( // left
            MathUtil.clamp(
                -yController.calculate(targetPose.getRotation().getY()),
                -1, 1)
        );

        if (Math.abs(targetPose.getRotation().getY()) < .1)
        {
            drive.withVelocityX( // forward
                -xController.calculate(targetPose.getZ())
            );
        }

        drivetrain.setControl(drive);
    }
}
