// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Ascender extends SubsystemBase {
  /** Creates a new Ascender. */
  //needs ID!!!
  private TalonFX ascendMotor = new TalonFX(1);
  public TalonFX wristMotor = new TalonFX(2);
  public TalonFX pivotMotorRight = new TalonFX(10);
  public TalonFX pivotMotorLeft = new TalonFX(11);
  public static CANrange caNrange = new CANrange(50);



  public PIDController ascendController = new PIDController(0.025, 0, 0.005);
  public PIDController wristController = new PIDController(0.25, 0, 0);
  public PIDController pivotController = new PIDController(0.1,0,0);

  public static double getRange() {
    return caNrange.getDistance().getValueAsDouble();
  }
  public TalonFXConfiguration talonConfig = new TalonFXConfiguration();
  public CurrentLimitsConfigs ascentCurrentLimit = new CurrentLimitsConfigs();
  public MotorOutputConfigs ascentConfig = new MotorOutputConfigs();
  //{Elevator Angle,Elevator Height,Wrist Angle}
  double preHeights[][] = {/*Down*/{0,0,0},/*Station*/{0,15,11},/*L3*/{0,17,37.1},/*L4*/{0,48,35},/*L2*/{0,0,36},/*Nothing*/{0,0,0}, /*Climbing*/{1,0,35}};

  double pivotRotations = 0;
  double wristRotations = 0;

  public Ascender() {
    ascentCurrentLimit.SupplyCurrentLimit = 20;
    ascentCurrentLimit.SupplyCurrentLimitEnable = true;
    ascentConfig.NeutralMode = NeutralModeValue.Brake;
    //ascendMotor.setPosition(0);
    //ascendMotor.curre
    //ascendMotorRight.MasterID(100).OpposeMasterDirection(false);
    ascendMotor.getConfigurator().apply(ascentConfig);
    ascendMotor.getConfigurator().apply(ascentCurrentLimit);
    pivotMotorLeft.getConfigurator().apply(ascentConfig);
    pivotMotorRight.getConfigurator().apply(ascentConfig);

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
  public void goToPosition(int position) {
    pivotController.setSetpoint(preHeights[position][0]);
    ascendController.setSetpoint(preHeights[position][1]);
    wristController.setSetpoint(preHeights[position][2]);
    System.out.println(position);
  }
  public void pivotControl(double addPose) {
    pivotController.setSetpoint(pivotController.getSetpoint() + addPose);
    System.out.println("hi");
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
    var wristSpeed = wristController.calculate(wristMotor.getPosition().getValueAsDouble());
    var pivotSpeed = pivotController.calculate((pivotMotorLeft.getPosition().getValueAsDouble() - pivotMotorRight.getPosition().getValueAsDouble()) / 2);
    SmartDashboard.putNumber("Ascender ascendSpeed Before Clamp", ascendSpeed);
    ascendSpeed = MathUtil.clamp(ascendSpeed, -0.25, 0.5);
    wristSpeed = MathUtil.clamp(wristSpeed, -0.5, 0.5);
    pivotSpeed = MathUtil.clamp(pivotSpeed, -2, 2);
    

    ascendMotor.set(ascendSpeed);
    wristMotor.set(wristSpeed);
    pivotMotorLeft.set(pivotSpeed);
    pivotMotorRight.set(-pivotSpeed);

    SmartDashboard.putNumber("WristPosition", wristMotor.getPosition().getValueAsDouble());
    

    SmartDashboard.putNumber("Ascender ascendSpeed After Clamp", ascendSpeed);
    SmartDashboard.putNumber("Ascender Error", ascendController.getError() * 1.621);
    SmartDashboard.putNumber("Ascend Encoder", ascendMotor.getPosition().getValueAsDouble()*1.62);
    SmartDashboard.putNumber("CanRange Output", caNrange.getDistance().getValueAsDouble()*39.3701);
    SmartDashboard.putNumber("Filtered CanRange", filteredDist);
    SmartDashboard.putNumber("Pivot Left", pivotMotorLeft.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Right", -pivotMotorRight.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Average", (pivotMotorLeft.getPosition().getValueAsDouble() - pivotMotorRight.getPosition().getValueAsDouble()) / 2);
    SmartDashboard.putNumber("Pivot Error", pivotController.getError());
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
