// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class IWannaDumpSomeCoral extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  CommandSwerveDrivetrain drive;
  Boolean left;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IWannaDumpSomeCoral(CommandSwerveDrivetrain drive, Boolean left) {
    this.drive = drive;
    this.left = left;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d targetPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
    Translation3d targetTrans = targetPose.getTranslation();
    Rotation3d targetRot = targetPose.getRotation();
    SmartDashboard.putNumber("Target T X",targetTrans.getX());
    SmartDashboard.putNumber("Target T Y",targetTrans.getY());
    SmartDashboard.putNumber("Target T Z",targetTrans.getZ());
    SmartDashboard.putNumber("Target R X",targetRot.getX());
    SmartDashboard.putNumber("Target R Y",targetRot.getY());
    SmartDashboard.putNumber("Target R Z",targetRot.getZ());
    System.out.println(targetRot.getX());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}