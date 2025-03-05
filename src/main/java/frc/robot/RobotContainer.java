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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Aim;
import frc.robot.commands.AscendTo;
import frc.robot.commands.IWannaDumpSomeCoral;
import frc.robot.commands.MoveMeters;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Ascender;
import frc.robot.subsystems.Canrange;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.JankyDumper;

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
    private final CommandXboxController joystickOperater = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    //TEST BOT
    // private final MoveMeters moveMeterCommand = new MoveMeters(2, drivetrain);
    // private final JankyDumper jankyDumper = new JankyDumper();
    private final Canrange canRange = new Canrange();
    private final Ascender ascender = new Ascender();

    private final SendableChooser<Command> autoChooser;
    CANdle candle = new CANdle(40,"CCR8612");
    private final MoveMeters moveMeters = new MoveMeters(1,drivetrain);
    private final IWannaDumpSomeCoral aprilFollowLeft = new IWannaDumpSomeCoral(drivetrain, true);
    private final IWannaDumpSomeCoral aprilFollowRight = new IWannaDumpSomeCoral(drivetrain, false);
    private final AscendTo ascendL1 = new AscendTo(ascender, 0);
    private final AscendTo ascendL2 = new AscendTo(ascender, 1);
    private final AscendTo ascendL3 = new AscendTo(ascender, 2);
    private final AscendTo ascendL4 = new AscendTo(ascender, 3);

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

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-Math.pow(joystickDrive.getLeftY(), 3) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-Math.pow(joystickDrive.getLeftX(), 3) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-Math.pow(joystickDrive.getRightX(), 3) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        joystickDrive.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystickDrive.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystickDrive.getLeftY(), -joystickDrive.getLeftX()))
        ));

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

        //joystickDrive.x().whileTrue(aprilFollowLeft);
        joystickDrive.b().whileTrue(aprilFollowRight);

        joystickOperater.a().whileTrue(ascendL1);
        joystickOperater.b().whileTrue(ascendL2);
        joystickOperater.x().whileTrue(ascendL3);
        joystickOperater.y().whileTrue(ascendL4);
    }
    

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
