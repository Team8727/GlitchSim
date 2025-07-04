// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  private final StringPublisher m_testStringPublisher 
    = edu.wpi.first.networktables.NetworkTableInstance.getDefault()
    .getStringTopic("test/string")
    .publish();
  private int m_counter = 0;

  public Robot() {}

  @Override
  public void robotPeriodic() {
    m_testStringPublisher.set("Hello, NetworkTables!" + (m_counter++));
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
