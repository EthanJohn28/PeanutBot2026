package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class Robot extends TimedRobot {

  // ---------------- MOTORS ----------------
  private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);

  // ---------------- CONTROLLER ----------------
  private final XboxController m_driverController = new XboxController(0);

  // ---------------- ENCODERS ----------------
  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);

  // ---------------- PID ----------------
  private final PIDController m_leftPID = new PIDController(0.1, 0.0, 0.0);
  private final PIDController m_rightPID = new PIDController(0.1, 0.0, 0.0);

  // ---------------- PHYSICAL CONSTANTS ----------------
  private final double WHEEL_DIAMETER_METERS = 0.1524; // 6 in wheel
  private final double ENCODER_TICKS_PER_REV = 2048;   // CHANGE if needed
  private final double GEAR_RATIO = 10.71;

  private final double metersPerTick;

  public Robot() {

    // ---------------- LOGGING (AdvantageScope) ----------------
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    SendableRegistry.addChild(m_leftMotor, m_leftMotor);
    SendableRegistry.addChild(m_rightMotor, m_rightMotor);

    m_rightMotor.setInverted(true);

    // ---------------- DASHBOARD GAINS ----------------
    SmartDashboard.putNumber("Drive/kP", 0.1);
    SmartDashboard.putNumber("Drive/kI", 0.0);
    SmartDashboard.putNumber("Drive/kD", 0.0);
    SmartDashboard.putNumber("Drive/MaxSpeed_mps", 3.0);

    // ---------------- ENCODER SETUP ----------------
    double wheelCircumference = Math.PI * WHEEL_DIAMETER_METERS;
    metersPerTick = wheelCircumference / (ENCODER_TICKS_PER_REV * GEAR_RATIO);

    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  @Override
  public void teleopPeriodic() {

    // ---------------- READ GAINS ----------------
    double kP = SmartDashboard.getNumber("Drive/kP", 0.1);
    double kI = SmartDashboard.getNumber("Drive/kI", 0.0);
    double kD = SmartDashboard.getNumber("Drive/kD", 0.0);

    m_leftPID.setPID(kP, kI, kD);
    m_rightPID.setPID(kP, kI, kD);

    // ---------------- TARGET SPEED (m/s) ----------------
    double maxSpeed = SmartDashboard.getNumber("Drive/MaxSpeed_mps", 3.0);

    double leftTarget = -m_driverController.getLeftY() * maxSpeed;
    double rightTarget = -m_driverController.getRightY() * maxSpeed;

    // ---------------- ACTUAL SPEED (m/s) ----------------
    double leftSpeed = m_leftEncoder.getRate() * metersPerTick;
    double rightSpeed = m_rightEncoder.getRate() * metersPerTick;

    // ---------------- PID OUTPUT ----------------
    double leftOutput = m_leftPID.calculate(leftSpeed, leftTarget);
    double rightOutput = m_rightPID.calculate(rightSpeed, rightTarget);

    // ---------------- SAFETY CLAMP ----------------
    leftOutput = Math.max(-1.0, Math.min(1.0, leftOutput));
    rightOutput = Math.max(-1.0, Math.min(1.0, rightOutput));

    // ---------------- DRIVE ----------------
    m_leftMotor.set(leftOutput);
    m_rightMotor.set(rightOutput);

    // ---------------- ADVANTAGESCOPE TELEMETRY ----------------
    SmartDashboard.putNumber("Drive/LeftTarget_mps", leftTarget);
    SmartDashboard.putNumber("Drive/RightTarget_mps", rightTarget);

    SmartDashboard.putNumber("Drive/LeftSpeed_mps", leftSpeed);
    SmartDashboard.putNumber("Drive/RightSpeed_mps", rightSpeed);

    SmartDashboard.putNumber("Drive/LeftError", leftTarget - leftSpeed);
    SmartDashboard.putNumber("Drive/RightError", rightTarget - rightSpeed);

    SmartDashboard.putNumber("Drive/LeftOutput", leftOutput);
    SmartDashboard.putNumber("Drive/RightOutput", rightOutput);

    SmartDashboard.putNumber("Drive/kP_live", kP);
    SmartDashboard.putNumber("Drive/kI_live", kI);
    SmartDashboard.putNumber("Drive/kD_live", kD);
  }
}