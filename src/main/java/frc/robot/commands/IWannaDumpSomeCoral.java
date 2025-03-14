// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class IWannaDumpSomeCoral extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  CommandSwerveDrivetrain drive;
  boolean offsetLeft = true;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IWannaDumpSomeCoral(CommandSwerveDrivetrain drive, boolean left) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    if(left == true) {
      offsetLeft = false;
    } else {
      offsetLeft = true;
    }
    addRequirements(drive);
    stage = 0;
  }
  private final SwerveRequest.FieldCentric IWannaFieldRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.RobotCentric IWannaRobotRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  PIDController leftright = new PIDController(3,0,0);
  PIDController turny = new PIDController(3,0,0);
  PIDController forward = new PIDController(1,0,0);
  PIDController offXPID = new PIDController(1,0.5,0);
  PIDController offYPID = new PIDController(1,0.5,0);
  SlewRateLimiter restraint_wehavenone = new SlewRateLimiter(3);
  SlewRateLimiter leftRightLimit = new SlewRateLimiter(5);
  SlewRateLimiter forwardLimit = new SlewRateLimiter(3);
  SlewRateLimiter offXLimit = new SlewRateLimiter(3);
  SlewRateLimiter offYLimit = new SlewRateLimiter(3);
  int stage = 0;
  double aprilAngle = 0;
  double offsetFieldX = 0;
  double offsetFieldY = 0;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d targetPoseRight = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-right");
    Pose3d targetPoseLeft = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-left");
    Translation3d targetTrans = targetPoseLeft.getTranslation();
    if(!offsetLeft) {
      targetTrans = targetPoseRight.getTranslation();
    }
    
    Rotation3d targetRot = targetPoseLeft.getRotation();
    if(!offsetLeft) {
      targetRot = targetPoseRight.getRotation();
    }
    System.out.println(targetPoseRight);
    SmartDashboard.putNumber("Target T X",targetTrans.getX());
    SmartDashboard.putNumber("Target T Y",targetTrans.getY());
    SmartDashboard.putNumber("Target T Z",targetTrans.getZ());
    SmartDashboard.putNumber("Target R X",targetRot.getX());
    SmartDashboard.putNumber("Target R Y",targetRot.getY());
    SmartDashboard.putNumber("Target R Z",targetRot.getZ());
    double xError = targetTrans.getX();
    double yawError = targetRot.getY();
    double zError = targetTrans.getZ() - 0.3;
    SmartDashboard.putNumber("xError", xError);
    SmartDashboard.putNumber("yawError", yawError);
    SmartDashboard.putNumber("zError", zError);
    SmartDashboard.putNumber("Stage 1 Error", (Math.abs(yawError) + Math.abs(xError)));
    if(stage == 0) {
      //move to center and turn
      var tVelocity = turny.calculate(xError);
      var yVelocity = leftright.calculate(-yawError);
      SmartDashboard.putNumber("Turning",tVelocity);
      SmartDashboard.putNumber("Moving",yVelocity);
      tVelocity = restraint_wehavenone.calculate(tVelocity);
      yVelocity = leftRightLimit.calculate(yVelocity);
      //Slight Forward
      var xVelocity = forward.calculate(zError);
      xVelocity = MathUtil.clamp(forwardLimit.calculate(xVelocity), -0.1, 0.1);
      drive.setControl(IWannaRobotRequest.withVelocityY(yVelocity).withRotationalRate(tVelocity).withVelocityX(-xVelocity));
      if(Math.abs(yawError) + Math.abs(xError) <= 0.05) {
        stage += 1;
      }
      System.out.println(xError);
      System.out.println(yawError);
    }
    else if(stage == 1) {
      //move forward
      var xVelocity = forward.calculate(zError);
      xVelocity = forwardLimit.calculate(xVelocity);
      //Slight Centering
      var tVelocity = turny.calculate(xError);
      var yVelocity = leftright.calculate(-yawError);
      tVelocity = MathUtil.clamp(restraint_wehavenone.calculate(tVelocity), -0.5, 0.5);
      yVelocity = MathUtil.clamp(leftRightLimit.calculate(yVelocity), -0.5, 0.5);
      drive.setControl(IWannaRobotRequest.withVelocityX(-xVelocity).withVelocityY(yVelocity).withRotationalRate(tVelocity));
      System.out.println(zError);
      if(Math.abs(zError) <= 0.02) {
        stage += 1;
      }
    }
    System.out.println(stage);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stage = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stage == 2;
    //return false;
  }
}