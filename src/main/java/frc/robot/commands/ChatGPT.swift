package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReefSwerveCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final NetworkTable limelightLeft;
    private final NetworkTable limelightRight;
    private final boolean offsetRight;

    private static final double SCORING_OFFSET_METERS = 0.2;  // Adjust per field measurements
    private static final double TARGET_DISTANCE = 1.0; // Stop this far from the reef
    private static final double ALIGN_TOLERANCE_DEGREES = 1.5;
    private static final double DRIVE_TOLERANCE_METERS = 0.05;
    private static final double KP_STRAFE = 0.5;  // Strafe correction multiplier
    private static final double KP_DRIVE = 0.4;   // Forward drive multiplier
    private static final double KP_ROTATE = 0.02; // Rotation multiplier

    public AlignToReefSwerveCommand(CommandSwerveDrivetrain drivetrain, String leftLimelightName, String rightLimelightName, boolean offsetRight) {
        this.drivetrain = drivetrain;
        this.limelightLeft = NetworkTableInstance.getDefault().getTable(leftLimelightName);
        this.limelightRight = NetworkTableInstance.getDefault().getTable(rightLimelightName);
        this.offsetRight = offsetRight;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("Aligning to Reef (Swerve)...");
    }

    @Override
    public void execute() {
        double[] virtualPose = getVirtualBotPose();
        double x = virtualPose[0];  // Sideways position
        double zDistance = virtualPose[2];  // Forward distance
        double yaw = virtualPose[5];  // Rotation

        // Compute left/right scoring offset correction
        double scoringOffset = offsetRight ? SCORING_OFFSET_METERS : -SCORING_OFFSET_METERS;

        // Target yaw angle for alignment
        double scoringOffsetAngle = Math.toDegrees(Math.atan(scoringOffset / zDistance));
        double targetYaw = yaw + scoringOffsetAngle;

        // Calculate movement values
        double strafeSpeed = KP_STRAFE * (x + scoringOffset); // Correct left/right drift
        double driveSpeed = KP_DRIVE * (zDistance - TARGET_DISTANCE); // Drive forward/back
        double turnSpeed = -KP_ROTATE * (yaw - targetYaw); // Rotate to align

        // Create field-oriented chassis speeds
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            -driveSpeed,  // Forward/backward
            -strafeSpeed, // Left/right
            turnSpeed,    // Rotation
            Rotation2d.fromDegrees(drivetrain.getYaw()) // Use gyro heading for field-relative control
        );

        // Command drivetrain to move
        drivetrain.drive(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        System.out.println("Aligned to Reef (Swerve)!");
    }

    @Override
    public boolean isFinished() {
        double[] virtualPose = getVirtualBotPose();
        double zDistance = virtualPose[2];
        double yaw = virtualPose[5];

        double scoringOffset = offsetRight ? SCORING_OFFSET_METERS : -SCORING_OFFSET_METERS;
        double scoringOffsetAngle = Math.toDegrees(Math.atan(scoringOffset / zDistance));
        double targetYaw = yaw + scoringOffsetAngle;

        return (Math.abs(yaw - targetYaw) <= ALIGN_TOLERANCE_DEGREES) &&
               (Math.abs(zDistance - TARGET_DISTANCE) <= DRIVE_TOLERANCE_METERS);
    }

    private double[] getVirtualBotPose() {
        double[] poseLeft = limelightLeft.getEntry("botpose").getDoubleArray(new double[6]);
        double[] poseRight = limelightRight.getEntry("botpose").getDoubleArray(new double[6]);

        if (poseLeft.length < 6 || poseRight.length < 6) {
            return new double[]{0, 0, 0, 0, 0, 0}; // Fail-safe
        }

        double xLeft = poseLeft[0], zLeft = poseLeft[2], yawLeft = poseLeft[5];
        double xRight = poseRight[0], zRight = poseRight[2], yawRight = poseRight[5];

        double txLeft = limelightLeft.getEntry("tx").getDouble(0.0);
        double txRight = limelightRight.getEntry("tx").getDouble(0.0);
        double weightLeft = 1.0 / (Math.abs(txLeft) + 1.0);
        double weightRight = 1.0 / (Math.abs(txRight) + 1.0);

        double xAvg = (xLeft * weightLeft + xRight * weightRight) / (weightLeft + weightRight);
        double zAvg = (zLeft * weightLeft + zRight * weightRight) / (weightLeft + weightRight);
        double yawAvg = (yawLeft * weightLeft + yawRight * weightRight) / (weightLeft + weightRight);

        return new double[]{xAvg, 0, zAvg, 0, 0, yawAvg};
    }
}
