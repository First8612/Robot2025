// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Aim;
import frc.robot.commands.IWannaDumpSomeCoral;
import frc.robot.commands.MoveMeters;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Ascender;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StabbyThingy;
import frc.robot.subsystems.AlgaeExtender;
import frc.robot.subsystems.AlgaeGrabber;
//import frc.robot.subsystems.Pivot;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystickDrive = new CommandXboxController(0);
    private final CommandXboxController joystickOperator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    //TEST BOT
    // private final MoveMeters moveMeterCommand = new MoveMeters(2, drivetrain);
    // private final JankyDumper jankyDumper = new JankyDumper();
    private final Ascender ascender = new Ascender();
    private final StabbyThingy stabber = new StabbyThingy();
    private final AlgaeExtender extender = new AlgaeExtender();
    private final AlgaeGrabber algaeRoll = new AlgaeGrabber();

    private final SendableChooser<Command> autoChooser;
    CANdle candle = new CANdle(40);
   // private final MoveMeters moveMeters = new MoveMeters(1,drivetrain);
    private final IWannaDumpSomeCoral aprilFollowLeft = new IWannaDumpSomeCoral(drivetrain, true);
    private final IWannaDumpSomeCoral aprilFollowRight = new IWannaDumpSomeCoral(drivetrain, false);

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
        
        configureBindings();

        drivetrain.configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser("MoveMeter Auto");
        
        SmartDashboard.putData("Auto Path", autoChooser);
    }

    void changeSpeed(double newSpeed) {
        speedMulti = newSpeed;
    }

    private void configureBindings() {
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
        //joystickDrive.rightBumper().whileTrue(jankyDumper.createDumpCommand());
        // joystickDrive.rightTrigger().toggleOnTrue(new Aim(drivetrain));

        // joystickDrive.button(2).onTrue(Commands.runOnce(SignalLogger::start));
        // joystickDrive.button(1).onTrue(Commands.runOnce(SignalLogger::stop));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystickDrive.rightBumper().and(joystickDrive.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystickDrive.rightBumper().and(joystickDrive.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystickDrive.leftBumper().and(joystickDrive.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystickDrive.leftBumper().and(joystickDrive.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystickDrive.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        //joystickDrive.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);

        joystickDrive.x().onTrue(aprilFollowLeft);
        joystickDrive.b().onTrue(aprilFollowRight);

        joystickOperator.povRight().onTrue(new InstantCommand(() -> ascender.goToPosition(0), ascender));
        joystickOperator.povDown().onTrue(new InstantCommand(() -> ascender.goToPosition(4), ascender));
        joystickOperator.povLeft().onTrue(new InstantCommand(() -> ascender.goToPosition(2), ascender));
        joystickOperator.povUp().onTrue(new InstantCommand(() -> ascender.goToPosition(3), ascender));
        joystickOperator.y().onTrue(new InstantCommand(() -> ascender.goToPosition(1), ascender));
        joystickOperator.rightBumper().onTrue(new InstantCommand(() -> ascender.goToPosition(6), ascender));

        joystickOperator.leftBumper().onTrue(new InstantCommand(() -> extender.setAlgae(extender.goToPosition)));

        joystickOperator.leftTrigger().whileTrue(new RunCommand(() -> algaeRoll.runAlgaeIn(joystickOperator.getLeftTriggerAxis()/2)));
        joystickOperator.rightTrigger().whileTrue(new RunCommand(() -> algaeRoll.runAlgaeIn(-joystickOperator.getRightTriggerAxis()/2)));
        joystickOperator.rightTrigger().onFalse(new InstantCommand(() -> algaeRoll.runAlgaeIn(0)));
        joystickOperator.leftTrigger().onFalse(new InstantCommand(() -> algaeRoll.runAlgaeIn(0)));
        joystickOperator.rightBumper().onTrue(new InstantCommand(() -> algaeRoll.runAlgaeIn(0)));

        joystickOperator.button(7).whileTrue(new RunCommand(() -> stabber.inFork(-0.1, true)));
        joystickOperator.button(7).whileFalse(new RunCommand(() -> stabber.inFork(0.1, false)));
        joystickOperator.a().whileTrue(new RunCommand(() -> stabber.inFork(0.25, true)));
        joystickOperator.a().whileFalse(new RunCommand(() -> stabber.inFork(0.1, stabber.overrider)));


        stabber.inFork(0.1, false);
    }

    public void autonomousInit() {
        //ascender.goToPosition(0);
        extender.setAlgae(0);
    }
    public void teleopInit() {
        //ascender.goToPosition(0);
        extender.setAlgae(0);
        ascender.pivotMotorLeft.setPosition(0);
        ascender.pivotMotorRight.setPosition(0);
    }
    public void teleopPeriodic() {
        ascender.pivotControl(joystickOperator.getRightY());
    }
    

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
