// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Ascender;
import frc.robot.subsystems.Wrist;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** An example command that uses an example subsystem. */
public class AscendTo extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Ascender ascender = new Ascender();
  private Wrist wrist = new Wrist();
  double pivotRotations = 0;
  double elevatorRotations = 0;
  double wristRotations = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AscendTo(Ascender subsystem, int preHeight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  SlewRateLimiter ascendLimit = new SlewRateLimiter(1);
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //init stuff
  }
  double ascendError = 0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ascendError = elevatorRotations - ascender.ascendMotor.getPosition().getValueAsDouble();
    var ascendVelocity = -ascendControl.calculate(ascendError);
    ascendVelocity = MathUtil.clamp(ascendLimit.calculate(ascendVelocity),-0.1,0.1);

    ascender.ascendMotor.set(ascendVelocity);
    SmartDashboard.putNumber("Ascender Error", ascendError * 1.621);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var fin = false;
    if(Math.abs(ascendError)<=0.1 && ascendError != 0) {
      fin = true;
    }
    return fin;
  }
}
