// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StabbyThingy extends SubsystemBase {
  public TalonFX forkMotor = new TalonFX(20);
  public CANrange forkDetector = new CANrange(51);
  /** Creates a new StabbyThingy. */
  public boolean overrider = false;
  public StabbyThingy() {

  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  double alpha = 0.1;
  double filteredDist = 0;
  public double noNoise() {

    double rawDist = forkDetector.getDistance().getValueAsDouble();
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
    //System.out.println(overrider);
    SmartDashboard.putNumber("Fork Detector", noNoise());
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
