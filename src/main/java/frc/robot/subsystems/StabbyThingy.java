// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StabbyThingy extends SubsystemBase {
  private TalonFX forkMotor = new TalonFX(20);
  private CANrange forkDetector = new CANrange(51);
  private MotorOutputConfigs forkConfig = new MotorOutputConfigs();
  public boolean overrider = false;
  private CANdle candle = new CANdle(40);
  public StabbyThingy() {
    forkConfig.NeutralMode = NeutralModeValue.Brake;

    forkMotor.getConfigurator().apply(forkConfig);
  }

  private double alpha = 0.5;
  private double filteredDist = 0;

  private double noNoise() {

    double rawDist = forkDetector.getDistance().getValueAsDouble();
    SmartDashboard.putNumber("Fork/Detector/Raw", rawDist);

    filteredDist = alpha * rawDist + (1 - alpha) * filteredDist;
    return filteredDist;
  }

  public void inFork(double speed, boolean overriden) {
    if(filteredDist > 0.05 || overriden) {
      forkMotor.set(speed);
      if(filteredDist <= 0.05) {
        overriden = true;
      }
    } else {
      forkMotor.set(0);
      if(filteredDist > 0.05) {
        overriden = false;
      }
    }

  }

  public void feed(boolean out)
  {
    forkMotor.set(out ? 0.5 : -0.5);
  }

  public boolean getCoralPresent() {
    return filteredDist <= 0.05;
  }

  @Override
  public void periodic() {
    noNoise();
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Fork/Detector/Filtered", filteredDist);
    if(getCoralPresent()) {
      candle.setLEDs(0, 255, 0);
    }
    else{
      candle.setLEDs(255, 0, 0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

