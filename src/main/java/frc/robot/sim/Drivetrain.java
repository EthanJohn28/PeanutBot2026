package frc.robot.sim;

import static edu.wpi.first.units.Units.Meters;

import java.lang.reflect.Field;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Drivetrain {
    // These represent our regular encoder objects, which we would
    // create to use on a real robot.
    private Encoder m_leftEncoder = new Encoder(0, 1);
    private Encoder m_rightEncoder = new Encoder(2, 3);
    // These are our EncoderSim objects, which we will only use in
    // simulation. However, you do not need to comment out these
    // declarations when you are deploying code to the roboRIO.
    private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

    // Create our gyro object like we would on a real robot.
    private AnalogGyro m_gyro = new AnalogGyro(1);
    // Create the simulated gyro object, used for setting the gyro
    // angle. Like EncoderSim, this does not need to be commented out
    // when deploying code to the roboRIO.
    private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

    private PWMSparkMax m_leftMotor = new PWMSparkMax(0);
    private PWMSparkMax m_rightMotor = new PWMSparkMax(1);

    private Field2d m_fieldSim = new Field2d();

    private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0);

    // Create the simulation model of our drivetrain.
    private DifferentialDrivetrainSim m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
        KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
        KitbotGearing.k10p71,        // 10.71:1
        KitbotWheelSize.kSixInch,    // 6" diameter wheels.
        null                         // No measurement noise.
    ); 


    public Drivetrain() {
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * Constants.WHEEL_DIAMETER_METERS / Constants.ENCODER_TICKS_PER_REV);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * Constants.WHEEL_DIAMETER_METERS / Constants.ENCODER_TICKS_PER_REV);
    }
    public void simulationPeriodic() {
    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    m_driveSim.setInputs(m_leftMotor.get() * RobotController.getInputVoltage(),
                        m_rightMotor.get() * RobotController.getInputVoltage());
    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.02);
    // Update all of our sensors.
    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());

    }

    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
        m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();

    }

}