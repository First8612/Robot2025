// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StabbyThingy extends SubsystemBase {
  private TalonFX forkMotor = new TalonFX(20);
  private CANrange forkDetector = new CANrange(51);
  private MotorOutputConfigs forkConfig = new MotorOutputConfigs();
  public boolean overrider = false;

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
    if(noNoise() > 0.05 || overriden) {
      forkMotor.set(speed);
      if(noNoise() <= 0.05) {
        overriden = true;
      }
    } else {
      forkMotor.set(0);
      if(noNoise() > 0.05) {
        overriden = false;
      }
    }

    SmartDashboard.putNumber("Fork/Detector/Filtered", noNoise());
  }

  public void feed(boolean out)
  {
    forkMotor.set(out ? 0.5 : -0.5);
  }

  public boolean getCoralPresent() {
    return noNoise() <= 0.05;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

