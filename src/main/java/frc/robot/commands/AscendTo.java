// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Ascender;
import frc.robot.subsystems.Wrist;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** An example command that uses an example subsystem. */
public class AscendTo extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Ascender ascender = new Ascender();
  private Wrist wrist = new Wrist();
  double preHeights[][] = {{1,1,1}};
  double pivotRotations = 0;
  double elevatorRotations = 0;
  double wristRotations = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public void AscenderCommand(Ascender subsystem, int preHeight) {
    pivotRotations = preHeights[preHeight][0];
    elevatorRotations = preHeights[preHeight][1];
    wristRotations = preHeights[preHeight][2];
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ascender.ascendMotorLeft.setPosition(pivotRotations);
    wrist.wristPID.setReference(wristRotations,ControlType.kPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
