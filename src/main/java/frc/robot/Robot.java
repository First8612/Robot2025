// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public Robot() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
      // TODO Auto-generated method stub
      m_robotContainer.robotInit();
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    m_robotContainer.robotPeriodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    super.autonomousInit();
    m_robotContainer.autonomousInit();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    super.teleopInit();
    m_robotContainer.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.teleopPeriodic();

  }
  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
