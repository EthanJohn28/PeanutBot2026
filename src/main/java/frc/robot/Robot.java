package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LoggedTunableNumber;

public class Robot extends LoggedRobot {

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

  private final LoggedTunableNumber kP_tunable = new LoggedTunableNumber("Tuning/kP");
  private final LoggedTunableNumber kI_tunable = new LoggedTunableNumber("Tuning/kI");
  private final LoggedTunableNumber kD_tunable = new LoggedTunableNumber("Tuning/kD");
;

  public Robot() {

    SendableRegistry.addChild(m_leftMotor, m_leftMotor);
    SendableRegistry.addChild(m_rightMotor, m_rightMotor);

    m_rightMotor.setInverted(true);

    // ---------------- ENCODER SETUP ----------------
    double wheelCircumference = Math.PI * WHEEL_DIAMETER_METERS;
    metersPerTick = wheelCircumference / (ENCODER_TICKS_PER_REV * GEAR_RATIO);

    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  @Override
  public void robotInit() {
    Logger.addDataReceiver(new NT4Publisher());
    Logger.addDataReceiver(new WPILOGWriter());

    Logger.start();

    SmartDashboard.putNumber("Drive/MaxSpeed_mps", 3.0);

    kP_tunable.initDefault(0.1);
    kI_tunable.initDefault(0.0);
    kD_tunable.initDefault(0.0);

    m_leftEncoder.setDistancePerPulse(metersPerTick);
    m_rightEncoder.setDistancePerPulse(metersPerTick);
  }

  @Override
  public void teleopPeriodic() {

    // ---------------- READ GAINS ----------------
    double kP = kP_tunable.get();
    double kI = kI_tunable.get();
    double kD = kD_tunable.get();

    m_leftPID.setPID(kP, kI, kD);
    m_rightPID.setPID(kP, kI, kD);

    // ---------------- TARGET SPEED (m/s) ----------------
    double maxSpeed = SmartDashboard.getNumber("Drive/MaxSpeed_mps", 3.0);

    double leftTarget = -m_driverController.getLeftY() * maxSpeed;
    double rightTarget = -m_driverController.getRightY() * maxSpeed;

    // ---------------- ACTUAL SPEED (m/s) ----------------
    double leftSpeed = m_leftEncoder.getRate();
    double rightSpeed = m_rightEncoder.getRate();

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

    Logger.recordOutput("Debug/TeleopRunning", true);
    Logger.recordOutput("Debug/LeftJoystick", m_driverController.getLeftY());
    Logger.recordOutput("Debug/RightJoystick", m_driverController.getRightY());
    
    Logger.recordOutput("Drive/LeftTarget_mps", leftTarget);
    Logger.recordOutput("Drive/RightTarget_mps", rightTarget);

    Logger.recordOutput("Drive/LeftSpeed_mps", leftSpeed);
    Logger.recordOutput("Drive/RightSpeed_mps", rightSpeed);
    Logger.recordOutput("Drive/LeftError", leftTarget - leftSpeed);
    Logger.recordOutput("Drive/RightError", rightTarget - rightSpeed);

    Logger.recordOutput("Drive/LeftOutput", leftOutput);
    Logger.recordOutput("Drive/RightOutput", rightOutput);
    
  }

}