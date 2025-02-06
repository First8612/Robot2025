package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

public class JankyDumper extends SubsystemBase{
    public SparkMax dumpy = new SparkMax(7, MotorType.kBrushless);
    // private PIDController dumpyPID = new PIDController(1, 0, 0);
    // private SlewRateLimiter dumpyLimit = new SlewRateLimiter(20);
    private RelativeEncoder dumpyEncoder = dumpy.getEncoder();
    private SparkClosedLoopController dumpysOwnPID = dumpy.getClosedLoopController();
    public JankyDumper() {
        SparkMaxConfig dumpyConfig = new SparkMaxConfig();

        dumpyConfig
            .idleMode(IdleMode.kBrake);
        // dumpyConfig.encoder
        //     .positionConversionFactor(1000)
        //     .velocityConversionFactor(1000);
        dumpyConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.1, 0.0, 0.0);
            
        dumpy.configure(dumpyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        dumpyEncoder.setPosition(0);
        dumpysOwnPID.setReference(0, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        // double position = dumpyEncoder.getPosition();
        // double velocity = dumpyPID.calculate(position);
        // velocity = dumpyLimit.calculate(velocity);
        // dumpy.setVoltage(velocity);
        // System.out.println(velocity);
        // SmartDashboard.putNumber("JankyDumper/Velocity",velocity);
        // SmartDashboard.putNumber("JankyDumper/Position",position);
        // SmartDashboard.putNumber("JankyDumper/Setpoint",dumpyPID.getSetpoint());
    }

    public Command createDumpCommand()
    {
        return new StartEndCommand(() -> {
            dumpysOwnPID.setReference(2, ControlType.kPosition);
            // dumpyPID.setSetpoint(2);
        }, () -> {
            dumpysOwnPID.setReference(0, ControlType.kPosition);
            // dumpyPID.setSetpoint(0);
        }, this);
    }
}
