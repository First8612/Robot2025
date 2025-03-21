// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.GoToPreset.GoToPresetDown;
import frc.robot.commands.GoToPreset.GoToPresetUp;


public class Ascender extends SubsystemBase {
  /** Creates a new Ascender. */
  //needs ID!!!
  private TalonFX ascendMotor = new TalonFX(1);
  public TalonFX wristMotor = new TalonFX(2);
  public TalonFX pivotMotorRight = new TalonFX(60);
  public TalonFX pivotMotorLeft = new TalonFX(61);
  public CANcoder pivotCANcoder = new CANcoder(14);
  public CANcoder wristCANcoder = new CANcoder(15);
  public static CANrange caNrange = new CANrange(50);

  public PIDController ascendController = new PIDController(0.025, 0, 0.005);
  public PIDController wristController = new PIDController(0.25, 0, 0);
  public PIDController pivotController = new PIDController(0.02,0,0);

  public Follower pivotFollower = new Follower(61, true);

  public static double getRange() {
    return caNrange.getDistance().getValueAsDouble();
  }
  public TalonFXConfiguration talonConfig = new TalonFXConfiguration();
  public CurrentLimitsConfigs ascentCurrentLimit = new CurrentLimitsConfigs();
  public CurrentLimitsConfigs pivotCurrentLimit = new CurrentLimitsConfigs();
  public MotorOutputConfigs ascentConfig = new MotorOutputConfigs();
  public MotorOutputConfigs noBrakeMode = new MotorOutputConfigs();
  /*
  Elevator Angle (UNITS),
  Elevator Height (INCHES),
  Wrist Angle (DEGREES)
  L1-L4 (and climbing) need to be programmed in
  */
  public double preHeights[][] = {/*Down*/{-2,0.3,161},/*Station*/{-20,21.5,161},/*L3*/{0,17,46},/*L4*/{-27,48,40},/*L2*/{0,0,46},/*Zero*/{20,2,144.5}, /*Climbing*/{0,0,43},/*L1*/{0,13,0}};

  double pivotRotations = 0;
  double wristRotations = 0;
  

  public Ascender() {
    ascentCurrentLimit.SupplyCurrentLimit = 20;
    ascentCurrentLimit.SupplyCurrentLimitEnable = true;
    pivotCurrentLimit.SupplyCurrentLimit = 30;
    pivotCurrentLimit.SupplyCurrentLimitEnable = true;
    ascentConfig.NeutralMode = NeutralModeValue.Brake;
    noBrakeMode.NeutralMode = NeutralModeValue.Coast;
    //pivotMotorRight.setControl();
    //ascendMotor.setPosition(0);
    //ascendMotor.curre
    //ascendMotorRight.MasterID(100).OpposeMasterDirection(false);
    ascendMotor.getConfigurator().apply(ascentConfig);
    ascendMotor.getConfigurator().apply(ascentCurrentLimit);
    pivotMotorLeft.getConfigurator().apply(ascentConfig);
    pivotMotorLeft.getConfigurator().apply(pivotCurrentLimit);
    pivotMotorRight.getConfigurator().apply(ascentConfig);
    pivotMotorRight.getConfigurator().apply(pivotCurrentLimit);
    //wristMotor.getConfigurator().apply(ascentConfig);
    wristMotor.getConfigurator().apply(noBrakeMode);
    pivotMotorRight.setControl(pivotFollower);
    
    //wristMotor.setPosition(0);
  }
  /* 
  public void setPivot(double position) {
    pivotController.setSetpoint(position);
    if(position == 0) {
      goToPivotPosition = defaultPivotPosition;
    } else {
      goToPivotPosition = 0;
    }
  }
  */
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void goToPositionPivot(int position) {
    pivotController.setSetpoint(preHeights[position][0]);
    SmartDashboard.putNumber("SetPoint/wrist setpoint", preHeights[position][0]);
  }
  public void goToPositionAscend(int position) {
    ascendController.setSetpoint(preHeights[position][1]);
  }
  public void goToPositionWrist(int position) {
    wristController.setSetpoint(preHeights[position][2] / 360);
  }
  public void pivotControl(double addPose) {
    pivotController.setSetpoint(pivotController.getSetpoint() + addPose);
  }

  public boolean isPivotAtPosition() {
    return Math.abs(pivotController.getError()) < 0.5;
  }
  public boolean isAscendAtPosition() {
    return Math.abs(ascendController.getError()) < 0.25;
  }
  public boolean isWristAtPosition() {
    return Math.abs(wristController.getError()) < 0.02;
  }
  public SequentialCommandGroup goToPosition(int position) {
    SmartDashboard.putNumber("Last Position", position);
    if(this.preHeights[position][1] < this.ascendController.getSetpoint()){
      GoToPresetDown presetDown = new GoToPresetDown(position, this);
      return presetDown;
    }
    else {
      GoToPresetUp presetUp = new GoToPresetUp(position, this);
      return presetUp;
    }
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  double alpha = 0.1;
  double filteredDist = 0;
  public double noNoise() {
    double rawDist = (caNrange.getDistance().getValueAsDouble() * 39.3701 - 2) * 2;
    filteredDist = alpha * rawDist + (1 - alpha) * filteredDist;
    return filteredDist;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var ascendSpeed = ascendController.calculate(noNoise());
    var wristSpeed = wristController.calculate(wristCANcoder.getAbsolutePosition().getValueAsDouble());
    var pivotSpeed = pivotController.calculate((pivotCANcoder.getAbsolutePosition().getValueAsDouble() - 0.27) * -800);
    SmartDashboard.putNumber("Ascend/Ascender ascendSpeed Before Clamp", ascendSpeed);
    ascendSpeed = MathUtil.clamp(ascendSpeed, -0.15, 0.5);
    wristSpeed = MathUtil.clamp(wristSpeed, -0.5, 0.5);
    pivotSpeed = MathUtil.clamp(pivotSpeed, -2.5, 2.5);
    

    ascendMotor.set(ascendSpeed);
    wristMotor.set(wristSpeed);
    pivotMotorLeft.set(pivotSpeed);

    SmartDashboard.putNumber("Wrist/WristPosition Motor", wristMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Wrist/Wrist Pos CANcoder", wristCANcoder.getAbsolutePosition().getValueAsDouble() * 360);

    SmartDashboard.putNumber("Ascend/Ascender ascendSpeed After Clamp", ascendSpeed);
    SmartDashboard.putNumber("Ascend/Ascender Error", ascendController.getError() * 1.621);
    SmartDashboard.putNumber("Ascend/Ascend Encoder", ascendMotor.getPosition().getValueAsDouble()*1.62);
    SmartDashboard.putNumber("Ascend/CanRange Output", caNrange.getDistance().getValueAsDouble()*39.3701);
    SmartDashboard.putNumber("Ascend/Filtered CanRange", filteredDist);
    SmartDashboard.putNumber("Pivot/Pivot Left", pivotMotorLeft.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Pivot/Pivot Right", pivotMotorRight.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Pivot/Pivot Average", (pivotMotorLeft.getPosition().getValueAsDouble() + pivotMotorRight.getPosition().getValueAsDouble()) / 2);
    SmartDashboard.putNumber("Pivot/Pivot Error", pivotController.getError());
    SmartDashboard.putNumber("Pivot/Pivot Left Current", pivotMotorLeft.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Pivot/Pivot Right Current", pivotMotorRight.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Pivot/Pivot/Encoder", pivotCANcoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Pivot/Adjusted Pivot", (pivotMotorLeft.getPosition().getValueAsDouble() + pivotMotorRight.getPosition().getValueAsDouble()) / -1600);
    SmartDashboard.putNumber("Pivot/Encoder Offset", pivotCANcoder.getAbsolutePosition().getValueAsDouble() - (pivotMotorLeft.getPosition().getValueAsDouble() + pivotMotorRight.getPosition().getValueAsDouble()) / -1600);
    SmartDashboard.putNumber("Pivot/Adjusted Encoder", (pivotCANcoder.getAbsolutePosition().getValueAsDouble() - 0.27) * -800);
    SmartDashboard.putNumber("Pivot/Set Point", pivotController.getSetpoint());
    // Pose3d targetPoseRight = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-right");
    // Pose3d targetPoseLeft = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-left");
    // Translation3d targetTrans = targetPoseLeft.getTranslation();
    
    // Rotation3d targetRot = targetPoseLeft.getRotation();
    // SmartDashboard.putNumber("Target T X",targetTrans.getX());
    // SmartDashboard.putNumber("Target T Y",targetTrans.getY());
    // SmartDashboard.putNumber("Target T Z",targetTrans.getZ());
    // SmartDashboard.putNumber("Target R X",targetRot.getX());
    // SmartDashboard.putNumber("Target R Y",targetRot.getY());
    // SmartDashboard.putNumber("Target R Z",targetRot.getZ());
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
