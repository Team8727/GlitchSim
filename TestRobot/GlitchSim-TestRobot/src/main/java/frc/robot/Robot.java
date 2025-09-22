// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  private final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
  private final BooleanPublisher testBooleanPublisher = networkTableInstance.getBooleanTopic("test/boolean").publish();
  private final IntegerPublisher testIntegerPublisher = networkTableInstance.getIntegerTopic("test/integer").publish();
  private final FloatPublisher testFloatPublisher = networkTableInstance.getFloatTopic("test/float").publish();
  private final DoublePublisher testDoublePublisher = networkTableInstance.getDoubleTopic("test/double").publish();
  
  private double counter = 0.0;
  private final double increment = 0.23;

  public Robot() {}

  @Override
  public void robotPeriodic() {
    counter += increment;
    testBooleanPublisher.set(counter % 2.0 < 1.0);
    testIntegerPublisher.set((int) counter);
    testFloatPublisher.set((float) counter);
    testDoublePublisher.set(counter);
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
