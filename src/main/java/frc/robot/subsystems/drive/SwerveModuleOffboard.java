// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleOffboard {
  // Drive motor
  private final SparkMax m_driveMotor;
  private final RelativeEncoder m_driveEncoder;
  private final SparkClosedLoopController m_drivePID;

  // Turning motor
  private final SparkMax m_turningMotor;
  private final RelativeEncoder m_turningEncoder;
  private final SparkClosedLoopController m_turningPID;
  
  // Swerve module absolute encoder (wheel angle)
  private final CANcoder m_canCoder;
  private final double m_canCoderOffsetDegrees;

  SwerveModuleState state;
  private int DisplayCount = 0;

  /**
   * SwerveModuleOffboard - A SparkMax-based swerve module with canCoder wheel angle measurement
   *
   * @param driveMotorID The CAN ID of the drive motor.
   * @param turningMotorID The CAN ID of the turning motor.
   * @param magEncoderID The CAN ID of the magnetic encoder.
   * @param magEncoderOffsetDegrees The absolute offset of the magnetic encoder.
   */
  public SwerveModuleOffboard(int driveMotorID, int turningMotorID, int magEncoderID,
      double magEncoderOffsetDegrees) {
    // Create drive motor objects
    m_driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_drivePID = m_driveMotor.getClosedLoopController();

    // Create turning motor objects
    m_turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);
    m_turningEncoder = m_turningMotor.getEncoder();
    m_turningPID = m_turningMotor.getClosedLoopController();

    // Create steering encoder objects (high resolution encoder)
    m_canCoder = new CANcoder(magEncoderID, "rio");
    m_canCoderOffsetDegrees = magEncoderOffsetDegrees;

    configureDevices();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    if (SwerveConstants.DISABLE_ANGLE_OPTIMIZER) {
      // Only enable when correcting wheel offsets!
      if (DisplayCount++ > 40) {
        System.out.println("CAUTION: Steering optimizer is disabled!");
        DisplayCount = 0;
      }
    } else {
      // Optimize the reference state to avoid spinning further than 90 degrees
      desiredState.optimize(new Rotation2d(getAbsTurnAngle()));
    }

    state = desiredState;

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(new Rotation2d(getAbsTurnAngle())).getCos();

    // Set the PID reference states
    m_drivePID.setSetpoint(state.speedMetersPerSecond, (ConstantsOffboard.DRIVE_MOTOR_PROFILED_MODE) ? SparkMax.ControlType.kMAXMotionVelocityControl : SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    m_turningPID.setSetpoint(state.angle.getRadians(), (ConstantsOffboard.ANGLE_MOTOR_PROFILED_MODE) ? SparkMax.ControlType.kMAXMotionPositionControl : SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0.0);
    m_turningEncoder.setPosition(0.0);
  }

  public SwerveModuleState getState() {
    double velocity = m_driveEncoder.getVelocity();
    Rotation2d rot = new Rotation2d(getAbsTurnAngle());
    return new SwerveModuleState(velocity, rot);
  }

  /**
   * Returns the CANcoder's measured turn angle in degrees.
   */
  public double getCanCoder() {
    var posVal = m_canCoder.getPosition();
    if(posVal.getStatus().isOK()) {
        double val = posVal.getValue().in(Degrees);
        // return val * 360.0;
        return val;
    } else {
        /* Report error and retry later */
        System.out.println("Error reading CANcoder position! Robot will not drive straight!");
        return 0.0;
    }
  }
  /**
   * Returns the CANcoder's measured turn angle in degrees.
   */
  public double getAbsTurnAngle() {
    var posVal = m_canCoder.getAbsolutePosition(); // This actaully waits that long! Don't call after init!
    if(posVal.getStatus().isOK()) {
        /* Perform seeding */
        double val = posVal.getValue().in(Degrees);
        return (Units.degreesToRadians(val - m_canCoderOffsetDegrees));
    } else {
        /* Report error and retry later */
        System.out.println("Error reading CANcoder position! Robot will not drive straight!");
        return 0;
    }

  }

  /**
   * Returns the SparkMax internal encoder's measured turn angle in degrees.
   */
  public Rotation2d getAngle() {
    return new Rotation2d(getAbsTurnAngle());
  }

  /**
   * Returns the SparkMax internal encoder's measured position in meters.
   */
  public SwerveModulePosition getPosition() {
    double distance = m_driveEncoder.getPosition();
    Rotation2d rot = new Rotation2d(getAbsTurnAngle());
    return new SwerveModulePosition(distance, rot);
  }

  public double getVelocity() {
    return m_driveEncoder.getVelocity();
  }
  
  public double getCurrent() {
    return m_driveMotor.getOutputCurrent();
  }

   public double getTurnCurrent() {
    return m_driveMotor.getOutputCurrent();
  }

  private void configureDevices() {
    configureCanCoder();
    configureDriveMotor();
    configureTurningMotor();
  }

  // Configure CANcoder
  private void configureCanCoder() {
    /* Configure CANcoder */
    var toApply = new CANcoderConfiguration();
    toApply.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    toApply.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_canCoder.getConfigurator().apply(toApply);

     /* Speed up signals to an appropriate rate */
    m_canCoder.getPosition().setUpdateFrequency(100);
    m_canCoder.getVelocity().setUpdateFrequency(100);
  }

  //TODO: Fix PID tuning
  // Configure turning motor
  private void configureTurningMotor() {
    var m_turningMotorConfig = new SparkMaxConfig()
      .inverted(ConstantsOffboard.ANGLE_MOTOR_INVERSION)
      .smartCurrentLimit(ConstantsOffboard.ANGLE_CURRENT_LIMIT);

    if (ConstantsOffboard.ANGLE_MOTOR_PROFILED_MODE) {
      m_turningMotorConfig.closedLoop.pid(
      ConstantsOffboard.ANGLE_KP_PROFILED, 
      ConstantsOffboard.ANGLE_KI_PROFILED, 
      ConstantsOffboard.ANGLE_KD_PROFILED)
      .feedForward.kV(ConstantsOffboard.ANGLE_KF_PROFILED);
    } else {
      m_turningMotorConfig.closedLoop.pid(
      ConstantsOffboard.ANGLE_KP, 
      ConstantsOffboard.ANGLE_KI, 
      ConstantsOffboard.ANGLE_KD)
      .feedForward.kV(ConstantsOffboard.ANGLE_KF);
    }

    m_turningMotorConfig.closedLoop
      .positionWrappingEnabled(true)
      .positionWrappingMaxInput(2 * Math.PI)
      .positionWrappingMinInput(0)

      .maxMotion
        .cruiseVelocity(ConstantsOffboard.ANGLE_MAX_VEL_PROFILED)
        .maxAcceleration(ConstantsOffboard.ANGLE_MAX_ACC_PROFILED)
        .allowedProfileError(ConstantsOffboard.ANGLE_MAX_ERR_PROFILED);

    m_turningMotorConfig.encoder
      .positionConversionFactor(ConstantsOffboard.ANGLE_ROTATIONS_TO_RADIANS)
      .velocityConversionFactor(ConstantsOffboard.ANGLE_RPM_TO_RADIANS_PER_SECOND);

    m_turningMotor.configure(m_turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // According to this:
    // https://www.chiefdelphi.com/t/ctre-phoenix-pro-to-phoenix-6-looking-back-and-looking-ahead/437313/27
    // When using Phoenix6, the CanCoder should not have problems on startup as long as we wait for an update and check for errors.
    var posVal = m_canCoder.getAbsolutePosition().waitForUpdate(0.1); // This actaully waits that long! Don't call after init!
    if(posVal.getStatus().isOK()) {
        /* Perform seeding */
        double val = posVal.getValue().in(Degrees);
        m_turningEncoder.setPosition(Units.degreesToRadians(val - m_canCoderOffsetDegrees));
    } else {
        /* Report error and retry later */
        System.out.println("Error reading CANcoder position! Robot will not drive straight!");
    }

    // Math check!  Is reading the SparkMax position good enough?
    // CANcoder resolution in degrees: 360/4096 = 0.088 degrees
    // SparkMax encoder resolution: 360/ ( 48 positions * ANGLE_GEAR_RATIO (150/7) ) = 0.35 degrees
    // So is the CANcoder slightly better? Yes, but the CANcoder hardware spec:
    // https://store.ctr-electronics.com/content/user-manual/CANCoder%20User's%20Guide.pdf
    // says that the absolute position can be off by 1.44 degrees if there is rotation!
    // It may be possible to servo to m_canCoder.getPosition(), but then then CAN bus utilization will go way up.
    // Future project: Servo on the CANcoder position and see if CAN bus utilization is a problem.
  }

  //TODO: Fix PID tuning
  // Configure drive motor
  private void configureDriveMotor() {
    var m_turningMotorConfig = new SparkMaxConfig()
      .inverted(ConstantsOffboard.ANGLE_MOTOR_INVERSION)
      .smartCurrentLimit(ConstantsOffboard.DRIVE_CURRENT_LIMIT);
 
    if (ConstantsOffboard.DRIVE_MOTOR_PROFILED_MODE) {
      m_turningMotorConfig.closedLoop.pid(
        ConstantsOffboard.DRIVE_KP_PROFILED,
        ConstantsOffboard.DRIVE_KI_PROFILED,
        ConstantsOffboard.DRIVE_KD_PROFILED, ClosedLoopSlot.kSlot1)
        .feedForward.kV(ConstantsOffboard.DRIVE_KF_PROFILED, ClosedLoopSlot.kSlot1);
    } else {
      m_turningMotorConfig.closedLoop.pid(
        ConstantsOffboard.DRIVE_KP,
        ConstantsOffboard.DRIVE_KI,
        ConstantsOffboard.DRIVE_KD, ClosedLoopSlot.kSlot1)
        .feedForward.kV(ConstantsOffboard.DRIVE_KF, ClosedLoopSlot.kSlot1);
    }

    m_turningMotorConfig.closedLoop
      .maxMotion
        .cruiseVelocity(ConstantsOffboard.DRIVE_MAX_VEL_PROFILED, ClosedLoopSlot.kSlot1)
        .maxAcceleration(ConstantsOffboard.DRIVE_MAX_ACC_PROFILED, ClosedLoopSlot.kSlot1)
        .allowedProfileError(ConstantsOffboard.DRIVE_MAX_ERR_PROFILED, ClosedLoopSlot.kSlot1);

    m_turningMotorConfig.encoder
      .positionConversionFactor(ConstantsOffboard.DRIVE_ROTATIONS_TO_METERS)
      .velocityConversionFactor(ConstantsOffboard.DRIVE_RPM_TO_METERS_PER_SECOND);

    m_driveMotor.configure(m_turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_driveEncoder.setPosition(0);
  }
}