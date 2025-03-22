package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
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
    private PIDController xController = new PIDController(1, 0, 0);
    private PIDController yController = new PIDController(5, 0, 0);
    private PIDController rotationController = new PIDController(5, 0, 0);

    public AlignToTag(Limelight limelight, CommandSwerveDrivetrain drivetrain) {
        super();
        this.limelight = limelight;
        this.drivetrain = drivetrain;

        SmartDashboard.putData("AlignToTag/xController", xController);
        SmartDashboard.putData("AlignToTag/yController", yController);
        SmartDashboard.putData("AlignToTag/rotationController", rotationController);
    }

    @Override
    public void initialize() {
        xController.reset();
        rotationController.reset();
    }

    @Override
    public void execute() {
        System.out.println(limelight.getFiducialID());
        if (limelight.getFiducialID() == 0)
            return;

        var targetPose = limelight.getTargetPose3d_CameraSpace();
        SmartDashboard.putNumber("AlignToTag/targetPose/x", targetPose.getX());
        SmartDashboard.putNumber("AlignToTag/targetPose/y", targetPose.getY());
        SmartDashboard.putNumber("AlignToTag/targetPose/z", targetPose.getZ());

        drivetrain.setControl(robotDrive
            // .withCenterOfRotation(new Translation2d(1, 0))
            .withVelocityX( // forward
                -xController.calculate(targetPose.getZ())
            )
            // .withVelocityY( // left
            //     -yController.calculate(targetPose.getX())
            // )
            .withRotationalRate(
                rotationController.calculate(targetPose.getX())
            )
        );
    }
}
