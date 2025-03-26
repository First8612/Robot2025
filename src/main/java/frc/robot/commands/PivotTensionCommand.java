package frc.robot.commands;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Ascender;

public class PivotTensionCommand extends Command {
    private Follower followerConfig;
    private TalonFX pivotMotor;

    public PivotTensionCommand(
            TalonFX pivotMotor,
            Follower followerConfig,
            Ascender ascender) {
        super();
        this.pivotMotor = pivotMotor;
        this.followerConfig = followerConfig;
        addRequirements(ascender);
    }

    @Override
    public void initialize() {
        pivotMotor.setControl(new DutyCycleOut(0.1));
        System.out.println("PivotTensionCommand: Start");
    }

    @Override
    public void execute() {
        System.out.println("PivotTensionCommand: Executing");
    }

    @Override
    public boolean isFinished() {
        var current = pivotMotor.getStatorCurrent().getValueAsDouble();
        System.out.println("PivotTensionCommand: Stator current: " + current);
        return current > 20;
    }

    @Override
    public void end(boolean interrupted) {
        pivotMotor.setControl(followerConfig);
        System.out.println("PivotTensionCommand: Back to follower");
    }
}
