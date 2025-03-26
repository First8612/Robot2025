// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignToTag;
import frc.robot.commands.IWannaDumpSomeCoral;
import frc.robot.commands.MoveMeters;
import frc.robot.generated.TunerConstants;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Ascender;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StabbyThingy;
//import frc.robot.subsystems.AlgaeExtender;
//import frc.robot.subsystems.AlgaeGrabber;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystickDrive = new CommandXboxController(0);
    private final CommandXboxController joystickOperator = new CommandXboxController(1);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Ascender ascender = new Ascender();
    private final StabbyThingy stabber = new StabbyThingy();
    //private final AlgaeExtender extender = new AlgaeExtender();
    //private final AlgaeGrabber algaeRoll = new AlgaeGrabber();
    private final CANdle candle = new CANdle(40);

    private final SendableChooser<Command> autoChooser;
   
    // Commands
    private final IWannaDumpSomeCoral aprilFollowLeft = new IWannaDumpSomeCoral(drivetrain, true);
    private final IWannaDumpSomeCoral aprilFollowRight = new IWannaDumpSomeCoral(drivetrain, false);

    // Sensors
    private final Limelight limelightRight = new Limelight("limelight-right");
    private final Limelight limelightLeft = new Limelight("limelight-left");

    // NT
    StructPublisher<Pose2d> robotPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Poses/Robot", Pose2d.struct).publish();

    public double speedMulti = 1;

    public RobotContainer() {
        //commands for pathplanner
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 0.2; // dim the LEDs to half brightness
        candle.configAllSettings(config);
        candle.setLEDs(255, 255, 255);
        // StrobeAnimation rainbowAnim = new StrobeAnimation(255,0,0,0, 0.9, 64);
        // candle.animate(rainbowAnim);
        NamedCommands.registerCommand("MoveMeter", new MoveMeters(1,drivetrain));
        NamedCommands.registerCommand("ReefOffsetLeft", aprilFollowLeft);
        NamedCommands.registerCommand("ReefOffsetRight", aprilFollowRight);
        NamedCommands.registerCommand("L2 coral", new InstantCommand(() -> ascender.goToPosition(4), ascender));
        NamedCommands.registerCommand("L3 coral", new InstantCommand(() -> ascender.goToPosition(2), ascender));
        NamedCommands.registerCommand("L4 coral", new InstantCommand(() -> ascender.goToPosition(3), ascender));
        NamedCommands.registerCommand("Elevator Down", new InstantCommand(() -> ascender.goToPosition(0), ascender));
        NamedCommands.registerCommand("Station", new InstantCommand(() -> ascender.goToPosition(1), ascender));
        NamedCommands.registerCommand("L1 coral", new InstantCommand(() -> ascender.goToPosition(7), ascender));
        NamedCommands.registerCommand("Score Coral", new RunCommand(() -> stabber.inFork(0.25,true)));
        NamedCommands.registerCommand("Score Backfeed", new RunCommand(() -> stabber.inFork(-0.2,true)));

        
        configureBindings();

        drivetrain.configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser("MoveMeter Auto");
        
        SmartDashboard.putData("Auto Path", autoChooser);
    }

    void changeSpeed(double newSpeed) {
        speedMulti = newSpeed;
    }

    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-Math.pow(joystickDrive.getLeftY(), 3) * MaxSpeed * speedMulti) // Drive forward with negative Y (forward)
                    .withVelocityY(-Math.pow(joystickDrive.getLeftX(), 3) * MaxSpeed * speedMulti) // Drive left with negative X (left)
                    .withRotationalRate(-Math.pow(joystickDrive.getRightX(), 3) * MaxAngularRate * speedMulti) // Drive counterclockwise with negative X (left)
            )
        );
        joystickDrive.a().whileTrue(drivetrain.applyRequest(() -> brake));
        
        joystickDrive.rightBumper().onTrue(new InstantCommand(() -> changeSpeed(0.5)));
        joystickDrive.rightBumper().whileFalse(new InstantCommand(() -> changeSpeed(1)));

        //TEST BOT STUFF
        // joystickDrive.rightBumper().and(joystickDrive.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystickDrive.rightBumper().and(joystickDrive.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystickDrive.leftBumper().and(joystickDrive.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystickDrive.leftBumper().and(joystickDrive.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystickDrive.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        //joystickDrive.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystickDrive.x().whileTrue(new AlignToTag(limelightRight, drivetrain));
        joystickDrive.b().whileTrue(new AlignToTag(limelightLeft, drivetrain));

        joystickOperator.povRight().onTrue(ascender.goToPosition(0));
        joystickOperator.povDown().onTrue(ascender.goToPosition(4));
        joystickOperator.povLeft().onTrue(ascender.goToPosition(2));
        joystickOperator.povUp().onTrue(ascender.goToPosition(3));
        joystickOperator.y().onTrue(ascender.goToPosition(1));
        joystickOperator.rightBumper().onTrue(ascender.goToPosition(6));
        //L1
        //joystickOperator.b().onTrue(ascender.goToPosition(7));

        //joystickOperator.leftBumper().onTrue(new InstantCommand(() -> extender.setAlgae(extender.goToPosition)));

//for algae intake that no longer exists
        //joystickOperator.leftTrigger().whileTrue(new RunCommand(() -> algaeRoll.runAlgaeIn(joystickOperator.getLeftTriggerAxis()/2)));
        //joystickOperator.rightTrigger().whileTrue(new RunCommand(() -> algaeRoll.runAlgaeIn(-joystickOperator.getRightTriggerAxis()/4)));
        //joystickOperator.rightTrigger().onFalse(new InstantCommand(() -> algaeRoll.runAlgaeIn(0)));
        //joystickOperator.leftTrigger().onFalse(new InstantCommand(() -> algaeRoll.runAlgaeIn(0)));
        //joystickOperator.rightBumper().onTrue(new InstantCommand(() -> algaeRoll.runAlgaeIn(0)));

        //joystickOperator.button(7).whileTrue(new RunCommand(() -> stabber.inFork(0.1, true)));
        //joystickOperator.button(7).whileFalse(new RunCommand(() -> stabber.inFork(0.1, false)));
        joystickOperator.a().whileTrue(new RunCommand(() -> stabber.inFork(-0.125, true)));
        joystickOperator.a().whileFalse(new RunCommand(() -> stabber.inFork(0.25, false)));
        joystickOperator.button(7).whileTrue(new RunCommand(() -> stabber.inFork(0.15, true)));
        joystickOperator.button(8).whileTrue(new RunCommand(() -> stabber.inFork(0.75, true)));


        stabber.inFork(0.1, false);
    }

    public void robotInit() {
    }
    public void robotPeriodic() {
        //limelightRight.updateOdometry(drivetrain);
        //limelightLeft.updateOdometry(drivetrain);

        // var state = drivetrain.getState();
        // robotPosePublisher.set(state.Pose, (long)state.Timestamp);


    }

    public void autonomousInit() {
        ascender.startPosFix();
        //extender.setAlgae(0);
    }
    public void teleopInit() {
        ascender.startPosFix();
        //extender.setAlgae(0);
        //ascender.pivotMotorLeft.setPosition(0);
        //ascender.pivotMotorRight.setPosition(0);
    }
    public void teleopPeriodic() {
        ascender.pivotControl(joystickOperator.getRightY());
    }
    

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
