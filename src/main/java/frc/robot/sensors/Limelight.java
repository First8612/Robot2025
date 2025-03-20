package frc.robot.sensors;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Limelight {
    private final String limelightName;
    private final StructPublisher<Pose2d> posePublisher; 
    private final StructPublisher<Pose2d> posePublisherMT2; 

    public Limelight(String name) {
        super();
        this.limelightName = name;
        this.posePublisher = NetworkTableInstance.getDefault().getStructTopic("Poses/" + name, Pose2d.struct).publish();
        this.posePublisherMT2 = NetworkTableInstance.getDefault().getStructTopic("Poses/" + name + "-MT2", Pose2d.struct).publish();
    }

    public void updateOdometry(CommandSwerveDrivetrain drivetrain) {
        boolean useMegaTag2 = false; // set to false to use MegaTag1
        boolean doRejectUpdate = false;

        if (useMegaTag2 == false) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                if (mt1.rawFiducials[0].ambiguity > .7) {
                    doRejectUpdate = true;
                }
                if (mt1.rawFiducials[0].distToCamera > 3) {
                    doRejectUpdate = true;
                }
            }
            if (mt1.tagCount == 0) {
                doRejectUpdate = true;
            }

            if (!doRejectUpdate) {
                drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                drivetrain.addVisionMeasurement(
                        mt1.pose,
                        Utils.fpgaToCurrentTime(mt1.timestampSeconds));
                posePublisher.set(mt1.pose);
            }
        } else if (useMegaTag2 == true) {
            var robotRotation = drivetrain.getState().Pose.getRotation().getDegrees();
            LimelightHelpers.SetRobotOrientation(limelightName,
                robotRotation, 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
            
            var gyroRate = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
            if (Math.abs(gyroRate) > 720) // if our angular velocity is greater than 720 degrees per second,
                                                  // ignore vision updates
            {
                doRejectUpdate = true;
            }
            if (mt2.tagCount == 0) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
                drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                drivetrain.addVisionMeasurement(
                        mt2.pose,
                        Utils.fpgaToCurrentTime(mt2.timestampSeconds));
                posePublisherMT2.set(mt2.pose);
            }
        }
    }

}
