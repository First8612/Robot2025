// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public PIDController ascendController = new PIDController(0.05, 0.02, 0.007);
  public PIDController wristController = new PIDController(0.01, 0, 0);
  public PIDController pivotController = new PIDController(0.02,0.01,0.002);

  public Follower pivotFollower = new Follower(61, true);

  public static double getRange() {
    return caNrange.getDistance().getValueAsDouble();
  }
  public TalonFXConfiguration talonConfig = new TalonFXConfiguration();
  public CurrentLimitsConfigs ascentCurrentLimit = new CurrentLimitsConfigs();
  private CurrentLimitsConfigs pivotCurrentLimit = new CurrentLimitsConfigs();
  public MotorOutputConfigs brakeModeConfig = new MotorOutputConfigs();
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
    /*L4*/{-23,40,180},
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
    brakeModeConfig.NeutralMode = NeutralModeValue.Brake;
    noBrakeMode.NeutralMode = NeutralModeValue.Coast;

    //pivotMotorRight.setControl();
    //ascendMotor.setPosition(0);
    //ascendMotor.curre
    //ascendMotorRight.MasterID(100).OpposeMasterDirection(false);
    ascendMotor.getConfigurator().apply(brakeModeConfig);
    ascendMotor.getConfigurator().apply(ascentCurrentLimit);
    ascendController.setIZone(2);
    
    /*** WRIST */
    wristMotor.getConfigurator().apply(new CurrentLimitsConfigs()
      .withStatorCurrentLimit(30)
      .withStatorCurrentLimitEnable(true));
    wristMotor.getConfigurator().apply(brakeModeConfig);

    /*** PIVOT ****/
    pivotCurrentLimit.StatorCurrentLimit = 30;
    pivotCurrentLimit.StatorCurrentLimitEnable = true;
    pivotController.setIZone(2);
    setPivotMoveMode();

    /*** TELEMETRY */
    SmartDashboard.putData("Ascender/Pivot PID", pivotController);
    SmartDashboard.putData("Ascender/Wrist PID", wristController);
    SmartDashboard.putData("Ascender/Ascend PID", ascendController);
    SmartDashboard.putData(this);
  }

  public void setPivotMoveMode() {
    // left
    pivotMotorLeft.getConfigurator().apply(brakeModeConfig);
    pivotMotorLeft.getConfigurator().apply(pivotCurrentLimit);

    // right
    pivotMotorRight.getConfigurator().apply(brakeModeConfig);
    pivotMotorRight.getConfigurator().apply(pivotCurrentLimit);
    pivotMotorRight.setControl(pivotFollower);

    SmartDashboard.putString("Pivot/Mode", "Move");

  }

  public void setPivotBrakeMode() {
    //left (same as move mode, but repeated since this method is used for initialization, and clarity doesn't hurt)
    pivotMotorLeft.getConfigurator().apply(brakeModeConfig);
    pivotMotorLeft.getConfigurator().apply(pivotCurrentLimit);

    // right
    // set current limit low, and constantly fight against the position of the left motor to take up slack
    pivotMotorRight.getConfigurator().apply(
      new CurrentLimitsConfigs()
        .withStatorCurrentLimit(5) // is this good?
        .withStatorCurrentLimitEnable(true)
    );
    pivotMotorRight.setControl(new VoltageOut(1)); // is this good?

    SmartDashboard.putString("Pivot/Mode", "Brake");
  }

  public Command getPivotTensionCommand() {
    // to be replaced with brake mode?
    return new PivotTensionCommand(pivotMotorRight, pivotFollower, this);
  }

  public void goToPositionPivot(int position) {
    setPivotMoveMode();
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
    setPivotMoveMode();
    pivotController.setSetpoint(Math.min(pivotController.getSetpoint() + addPose,105));
  }

  public boolean isPivotAtPosition() {
    return Math.abs(pivotController.getError()) < 2;
  }
  public boolean isAscendAtPosition() {
    return Math.abs(ascendController.getError()) < 0.7;
  }
  public boolean isAscendAbove() {
    return getRange() > 5;
  }
  public boolean isWristAtPosition() {
    return Math.abs(wristController.getError()) < 10;
  }
  public Command goToPosition(int position) {
    return Commands.sequence(
      Commands.runOnce(() -> setPivotMoveMode(), this),

      new ConditionalCommand(
      new GoToPresetDown(position, this),
        new ConditionalCommand(
          new GoToPresetFromBottom(position, this),
          new GoToPresetNormal(position, this),
          () -> this.ascendController.getSetpoint() <= 2),
      () -> this.preHeights[position][1] <= 2),

      Commands.runOnce(() -> setPivotBrakeMode(), this)
    );
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
    ascendController.setSetpoint(noNoise() + 1);
    wristController.setSetpoint(fixAbsEnc());
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

    // SmartDashboard.putNumber("Wrist/WristPosition Motor", wristMotor.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("Wrist/Wrist Pos CANcoder", fixAbsEnc());
    // SmartDashboard.putNumber("Wrist/Wrist error", wristController.getError());
    // SmartDashboard.putNumber("Wrist/Wrist setpoint", wristController.getSetpoint());
    SmartDashboard.putBoolean("Wrist/IsAtPosition", isWristAtPosition());

    // SmartDashboard.putNumber("Ascend/Ascender ascendSpeed After Clamp", ascendSpeed);
    // SmartDashboard.putNumber("Ascend/Ascender Error", ascendController.getError() * 1.621);
    // SmartDashboard.putNumber("Ascend/Ascend Encoder", ascendMotor.getPosition().getValueAsDouble()*1.62);
    SmartDashboard.putNumber("Ascend/CanRange Output", caNrange.getDistance().getValueAsDouble()*39.3701);
    SmartDashboard.putNumber("Ascend/Filtered CanRange", filteredDist);
    SmartDashboard.putBoolean("Ascend/IsAtPosition", isAscendAtPosition());
    // SmartDashboard.putNumber("Pivot/Pivot Left", pivotMotorLeft.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("Pivot/Pivot Right", pivotMotorRight.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("Pivot/Pivot Average", (pivotMotorLeft.getPosition().getValueAsDouble() + pivotMotorRight.getPosition().getValueAsDouble()) / 2);
    // SmartDashboard.putNumber("Pivot/Pivot Error", pivotController.getError());
    // SmartDashboard.putNumber("Pivot/Pivot Left Current", pivotMotorLeft.getSupplyCurrent().getValueAsDouble());
    // SmartDashboard.putNumber("Pivot/Pivot Right Current", pivotMotorRight.getSupplyCurrent().getValueAsDouble());
    // SmartDashboard.putNumber("Pivot/Pivot/Encoder", pivotCANcoder.getAbsolutePosition().getValueAsDouble());
    // SmartDashboard.putNumber("Pivot/Adjusted Pivot", (pivotMotorLeft.getPosition().getValueAsDouble() + pivotMotorRight.getPosition().getValueAsDouble()) / -1600);
    // SmartDashboard.putNumber("Pivot/Encoder Offset", pivotCANcoder.getAbsolutePosition().getValueAsDouble() - (pivotMotorLeft.getPosition().getValueAsDouble() + pivotMotorRight.getPosition().getValueAsDouble()) / -1600);
     SmartDashboard.putNumber("Pivot/Adjusted Encoder", (pivotCANcoder.getAbsolutePosition().getValueAsDouble() - 0.27) * -800);
    // SmartDashboard.putNumber("Pivot/Set Point", pivotController.getSetpoint());
    // SmartDashboard.putNumber("hasGoToPos", hasGoToPos);
    SmartDashboard.putBoolean("Pivot/IsAtPosition", isPivotAtPosition());
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
