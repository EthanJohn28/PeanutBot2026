// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with tank
 * steering and an Xbox controller.
 */
public class Robot extends TimedRobot {

  private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);

  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  private final XboxController m_driverController = new XboxController(0);

  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);

    m_rightMotor.setInverted(true);

    SmartDashboard.putNumber("Drive/kP", 0.1);
    SmartDashboard.putNumber("Drive/kI", 0.0);
    SmartDashboard.putNumber("Drive/kD", 0.0);
  }

  @Override
  public void teleopPeriodic() {

    double kP = SmartDashboard.getNumber("Drive/kP", 0.1);
    double kI = SmartDashboard.getNumber("Drive/kI", 0.0);
    double kD = SmartDashboard.getNumber("Drive/kD", 0.0);

    m_robotDrive.tankDrive(
      -m_driverController.getLeftY(),
      -m_driverController.getRightY()
    );
  }
}