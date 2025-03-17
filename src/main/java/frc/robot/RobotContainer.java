// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix.Util;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.PhotonVision.PhotonVisionConstants;
//import frc.robot.PhotonVision.AprilTagThing;
import frc.robot.PhotonVision.*;
import frc.robot.PhotonVision;
//import frc.robot.AprilTagThing;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystickDrive = new CommandXboxController(0);
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    //private final PhotonVision aprilTagThing = new PhotonVision.AprilTagThing.april();
    

    //private final PhotonVision.AprilTagThing.@aprilTagThing(null)();

    private final PhotonVision photon1Vision = new PhotonVision(PhotonVisionConstants.camera1, PhotonVisionConstants.camera1Transform)/*.AprilTagThing(true)*/;
    private final PhotonVision photon2Vision = new PhotonVision(PhotonVisionConstants.camera2, PhotonVisionConstants.camera2Transform);
    
    //PhotonVision.AprilTagThing aprilTagThing = photon1Vision.new ApriltagThing();

    //creates a PhotonVision list for use when adding the vision measurement
    public PhotonVision[] photonVision =  new PhotonVision[]{photon1Vision, photon2Vision};
    //public Pose2d[] poseList = new Pose2d[]{};
    public ArrayList<Pose2d> poseList = new ArrayList<>();

    private final Field2d field = new Field2d();
    

    public RobotContainer() {
        configureBindings();

        drivetrain.configureAutoBuilder();

        SmartDashboard.putData(field);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-Math.pow(joystickDrive.getLeftY(), 3) * MaxSpeed) // Drive
                                                                                                                     // forward
                                                                                                                     // with
                                                                                                                     // negative
                                                                                                                     // Y
                                                                                                                     // (forward)
                        .withVelocityY(-Math.pow(joystickDrive.getLeftX(), 3) * MaxSpeed) // Drive left with negative X
                                                                                          // (left)
                        .withRotationalRate(-Math.pow(joystickDrive.getRightX(), 3) * MaxAngularRate) // Drive
                                                                                                      // counterclockwise
                                                                                                      // with negative X
                                                                                                      // (left)
                ));

        joystickDrive.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystickDrive.b().onTrue(new InstantCommand(() -> {
            drivetrain.resetFieldOrientation();
        }));
        joystickDrive.rightTrigger().onTrue(

            new InstantCommand(() -> 
                //new AprilTagThing(true)
                photon1Vision.AprilTagThing(true)
                //AprilTagThing(true)
            )

        );

        drivetrain.registerTelemetry(logger::telemeterize);

        
    }

    public Command getAutonomousCommand() {
        return null;
    }

    StructPublisher<Pose2d> photonVisionPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Poses/PhotonVision", Pose2d.struct).publish();
    StructPublisher<Pose2d> robotPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Poses/Robot", Pose2d.struct).publish();
    StructPublisher<Pose2d> camera1PosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Poses/Cameras/" + PhotonVisionConstants.camera1, Pose2d.struct).publish();
    StructPublisher<Pose2d> camera2PosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Poses/Cameras/" + PhotonVisionConstants.camera2, Pose2d.struct).publish();
    StructPublisher<Pose2d> cameraPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Poses/Cameras/" + PhotonVisionConstants.camera2, Pose2d.struct).publish();

        //Initializes the poses we put into advantagescope because otherwise it complains
        Pose2d pose1 = null;
        Pose2d pose2 = null;

    public void robotPeriodic() {
        for (int i = 0; i < photonVision.length; i++){
        var visionEst = photonVision[i].getEstimatedGlobalPose();
        final int j = i;
        visionEst.ifPresent(
            est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = photonVision[j].getEstimationStdDevs();

                
                
                var pose = est.estimatedPose.toPose2d();

                drivetrain.addVisionMeasurement(
                    pose, 
                    Utils.fpgaToCurrentTime(est.timestampSeconds),
                    estStdDevs);
                if(j == 0){
                    pose1 = pose;
                }else if(j == 1){
                    pose2 = pose;
                }

                
                //pushes photonvisions overall pose to advantagescope
                field.getObject("Photon Vision").setPose(pose);
                photonVisionPosePublisher.set(pose);
                

                
            });

        //pushes the camera poses to advantagescope
        if(pose1 != null){
        camera1PosePublisher.set(pose1);
        }
        if(pose2 != null){
        camera2PosePublisher.set(pose2);
        }

        //pushes the robot pose to advantagescope
        field.setRobotPose(drivetrain.getPose());
        robotPosePublisher.set((drivetrain.getPose()));
        }
    }

}
