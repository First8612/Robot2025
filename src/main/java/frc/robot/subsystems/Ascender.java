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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.PivotTensionCommand;
import frc.robot.commands.GoToPreset.GoToPresetDown;
import frc.robot.commands.GoToPreset.GoToPresetFromBottom;
import frc.robot.commands.GoToPreset.GoToPresetNormal;


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

  public PIDController ascendController = new PIDController(0.04, 0, 0.005);
  public PIDController wristController = new PIDController(0.01, 0, 0);
  public PIDController pivotController = new PIDController(0.02,0.01,0.002);

  public Follower pivotFollower = new Follower(61, true);

  public static double getRange() {
    return caNrange.getDistance().getValueAsDouble();
  }
  public TalonFXConfiguration talonConfig = new TalonFXConfiguration();
  public CurrentLimitsConfigs ascentCurrentLimit = new CurrentLimitsConfigs();
  public CurrentLimitsConfigs pivotCurrentLimit = new CurrentLimitsConfigs();
  public MotorOutputConfigs ascentConfig = new MotorOutputConfigs();
  public MotorOutputConfigs noBrakeMode = new MotorOutputConfigs();
  private int hasGoToPos = 0;
  /*
  Elevator Angle (UNITS),
  Elevator Height (INCHES),
  Wrist Angle (DEGREES)
  L1-L4 (and climbing) need to be programmed in
  */
  
  public double preHeights[][] = {
    /*Down*/{-2,0.3,40},
    /*Station*/{-20,21.5,40},
    /*L3*/{-20,20,158},
    /*L4*/{-27,36,158},
    /*L2*/{-20,5.7,153},
    /*Zero*/{20,2,22}, 
    /*Climbing*/{19.5,2.3,233},
    /*L1*/{0,13,0}
  };

  double pivotRotations = 0;
  double wristRotations = 0;
  

  public Ascender() {
    ascentCurrentLimit.SupplyCurrentLimit = 20;
    ascentCurrentLimit.SupplyCurrentLimitEnable = true;
    ascentCurrentLimit.StatorCurrentLimit = 50;
    ascentCurrentLimit.StatorCurrentLimitEnable = true;
    pivotCurrentLimit.SupplyCurrentLimit = 30;
    pivotCurrentLimit.StatorCurrentLimit = 30;
    pivotCurrentLimit.StatorCurrentLimitEnable = true;
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

    pivotController.setIZone(2);
    
    //wristMotor.setPosition(0);
    SmartDashboard.putData("Ascender/Pivot PID", pivotController);
    SmartDashboard.putData("Ascender/Wrist PID", wristController);
    SmartDashboard.putData("Ascender/Ascend PID", ascendController);
  }

  public Command getPivotTensionCommand() {
    return new PivotTensionCommand(pivotMotorRight, pivotFollower, this);
  }

  public void goToPositionPivot(int position) {
    pivotController.setSetpoint(preHeights[position][0]);
    SmartDashboard.putNumber("SetPoint/wrist setpoint", preHeights[position][0]);
  }
  public void goToPositionAscend(int position) {
    ascendController.setSetpoint(preHeights[position][1]);
  }
  public void goToPositionWrist(int position) {
    wristController.setSetpoint(preHeights[position][2]);
  }
  public void pivotControl(double addPose) {
    pivotController.setSetpoint(pivotController.getSetpoint() + addPose);
  }

  public boolean isPivotAtPosition() {
    return Math.abs(pivotController.getError()) < 2;
  }
  public boolean isAscendAtPosition() {
    return Math.abs(ascendController.getError()) < 3;
  }
  public boolean isAscendAbove() {
    return getRange() > 5;
  }
  public boolean isWristAtPosition() {
    return Math.abs(wristController.getError()) < 10;
  }
  public ConditionalCommand goToPosition(int position) {
    return new ConditionalCommand(
      new GoToPresetDown(position, this),
        new ConditionalCommand(
          new GoToPresetFromBottom(position, this),
          new GoToPresetNormal(position, this),
          () -> this.ascendController.getSetpoint() <= 2),
      () -> this.preHeights[position][1] <= 2);
  }

  public static double map(double value, double rangeAStart, double rangeAEnd, double rangeBStart, double rangeBEnd) {
    return rangeBStart + (value - rangeAStart) * (rangeBEnd - rangeBStart) / (rangeAEnd - rangeAStart);
  }

  private double fixAbsEnc(){
    if(wristCANcoder.getAbsolutePosition().getValueAsDouble() < 0){
      return map(wristCANcoder.getAbsolutePosition().getValueAsDouble() * 360, -180,0,180,360);
    }
    else{
      return wristCANcoder.getAbsolutePosition().getValueAsDouble() * 360;
    }
  }

  public void startPosFix() {
    pivotController.setSetpoint((pivotCANcoder.getAbsolutePosition().getValueAsDouble() - 0.27) * -800);
    ascendController.setSetpoint(noNoise());
    wristController.setSetpoint(fixAbsEnc());
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
    var wristSpeed = wristController.calculate(fixAbsEnc());
    var pivotSpeed = pivotController.calculate((pivotCANcoder.getAbsolutePosition().getValueAsDouble() - 0.27) * -800);
    SmartDashboard.putNumber("Ascend/Ascender ascendSpeed Before Clamp", ascendSpeed);
    ascendSpeed = MathUtil.clamp(ascendSpeed, -0.15, 0.5);
    wristSpeed = MathUtil.clamp(wristSpeed, -0.5, 0.5);
    pivotSpeed = MathUtil.clamp(pivotSpeed, -2.5, 2.5);

    ascendMotor.set(ascendSpeed);
    wristMotor.set(wristSpeed);
    pivotMotorLeft.set(pivotSpeed);

    SmartDashboard.putNumber("Wrist/WristPosition Motor", wristMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Wrist/Wrist Pos CANcoder", fixAbsEnc());
    SmartDashboard.putNumber("Wrist/Wrist error", wristController.getError());
    SmartDashboard.putNumber("Wrist/Wrist setpoint", wristController.getSetpoint());

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
    SmartDashboard.putNumber("hasGoToPos", hasGoToPos);
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
