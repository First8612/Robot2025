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
  

  public PIDController ascendController = new PIDController(0.05, 0, 0.005);
  public PIDController wristController = new PIDController(0.25, 0, 0);

  public static double getRange() {
    return caNrange.getDistance().getValueAsDouble();
  }
  public TalonFXConfiguration talonConfig = new TalonFXConfiguration();
  public CurrentLimitsConfigs ascentCurrentLimit = new CurrentLimitsConfigs();
  public MotorOutputConfigs ascentConfig = new MotorOutputConfigs();
  
  double preHeights[][] = {{0,0,0},{1,15,11},{1,18,37.1},{1,0,16},{1,0,20},{1,0,0}};

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
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void goToPosition(int position) {
    pivotRotations = preHeights[position][0];
    ascendController.setSetpoint(preHeights[position][1]);
    wristController.setSetpoint(preHeights[position][2]);
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
    SmartDashboard.putNumber("Ascender ascendSpeed Before Clamp", ascendSpeed);
    ascendSpeed = MathUtil.clamp(ascendSpeed, -0.5, 0.5);
    wristSpeed = MathUtil.clamp(wristSpeed, -0.1, 0.1);

    ascendMotor.set(ascendSpeed);
    wristMotor.set(wristSpeed);
    SmartDashboard.putNumber("WristPosition", wristMotor.getPosition().getValueAsDouble());
    

    SmartDashboard.putNumber("Ascender ascendSpeed After Clamp", ascendSpeed);
    SmartDashboard.putNumber("Ascender Error", ascendController.getError() * 1.621);
    SmartDashboard.putNumber("CanRange Output", caNrange.getDistance().getValueAsDouble()*39.3701);
    SmartDashboard.putNumber("Filtered CanRange", filteredDist);
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
