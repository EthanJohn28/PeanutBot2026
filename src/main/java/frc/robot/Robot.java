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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;

public class Robot extends LoggedRobot {

  // hardware
  private final SparkMax m_leftMotor = new SparkMax(8, MotorType.kBrushless);
  private final SparkMax m_rightMotor = new SparkMax(19, MotorType.kBrushless);

  private final XboxController m_driverController = new XboxController(0);

  // constants
  private final double WHEEL_DIAMETER_METERS = 0.1524; // 6 in wheel
  private final double ENCODER_TICKS_PER_REV = 2048;   // CHANGE if needed
  private final double GEAR_RATIO = 10.71;

  private final double metersPerTick;

  // pid & pid telemetry
  private final PIDController m_leftPID = new PIDController(0.1, 0.0, 0.0);
  private final PIDController m_rightPID = new PIDController(0.1, 0.0, 0.0);

  private final LoggedTunableNumber kP_tunable = new LoggedTunableNumber("Tuning/kP");
  private final LoggedTunableNumber kI_tunable = new LoggedTunableNumber("Tuning/kI");
  private final LoggedTunableNumber kD_tunable = new LoggedTunableNumber("Tuning/kD");


  public Robot() {
    // encoder setup
    double wheelCircumference = Math.PI * WHEEL_DIAMETER_METERS;
    metersPerTick = wheelCircumference / (ENCODER_TICKS_PER_REV * GEAR_RATIO);

    EncoderConfig leftConfig = new EncoderConfig();
    EncoderConfig rightConfig = new EncoderConfig();

    leftConfig.positionConversionFactor(metersPerTick);
    rightConfig.positionConversionFactor(metersPerTick);

    SparkMaxConfig leftSparkConfig = new SparkMaxConfig();
    SparkMaxConfig rightSparkConfig = new SparkMaxConfig();

    leftSparkConfig.apply(leftConfig);
    rightSparkConfig.apply(rightConfig);

    rightSparkConfig.inverted(true);

    m_leftMotor.configure(leftSparkConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    m_rightMotor.configure(rightSparkConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
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

    
  }

  @Override
  public void teleopPeriodic() {

    // implement pid
    double kP = kP_tunable.get();
    double kI = kI_tunable.get();
    double kD = kD_tunable.get();

    m_leftPID.setPID(kP, kI, kD);
    m_rightPID.setPID(kP, kI, kD);

    // values to log
    double maxSpeed = SmartDashboard.getNumber("Drive/MaxSpeed_mps", 3.0);

    double leftTarget = -m_driverController.getLeftY() * maxSpeed;
    double rightTarget = -m_driverController.getRightY() * maxSpeed;

    // in m/s
    double leftSpeed = m_leftMotor.getEncoder().getVelocity() / 60.0 * metersPerTick; // convert from RPM to m/s
    double rightSpeed = m_rightMotor.getEncoder().getVelocity() / 60.0 * metersPerTick; // convert from RPM to m/s

    double leftOutput = m_leftPID.calculate(leftSpeed, leftTarget);
    double rightOutput = m_rightPID.calculate(rightSpeed, rightTarget);

    // safety clamp (to not blow anything up)
    leftOutput = Math.max(-1.0, Math.min(1.0, leftOutput));
    rightOutput = Math.max(-1.0, Math.min(1.0, rightOutput));

    // implements actual driving
    m_leftMotor.set(leftOutput);
    m_rightMotor.set(rightOutput);

    // telemetry

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