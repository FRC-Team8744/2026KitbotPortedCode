// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private StructPublisher<Pose2d> pose_publisher = NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish();
  private StructArrayPublisher<SwerveModuleState> swerve_publisher = NetworkTableInstance.getDefault().getStructArrayTopic("Swerve States", SwerveModuleState.struct).publish();

  private double offset_FL = 0;
  private double offset_RL = 0;
  private double offset_FR = 0;
  private double offset_RR = 0;
  
  private double m_DriverSpeedScale = 1.0;

  // Robot swerve modules
  private final SwerveModuleOffboard m_frontLeft;
  private final SwerveModuleOffboard m_rearLeft;
  private final SwerveModuleOffboard m_frontRight;
  private final SwerveModuleOffboard m_rearRight;

  // The imu sensor
  public final Multi_IMU m_imu = new Multi_IMU();
  
  // Odometry class for tracking robot pose
  private SwerveDriveOdometry m_odometry;

  // Create Field2d for robot and trajectory visualizations.
  private Field2d m_field;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    offset_FL = SwerveConstants.kFrontLeftMagEncoderOffsetDegrees;
    offset_RL = SwerveConstants.kRearLeftMagEncoderOffsetDegrees;
    offset_FR = SwerveConstants.kFrontRightMagEncoderOffsetDegrees;
    offset_RR = SwerveConstants.kRearRightMagEncoderOffsetDegrees;
  
    m_frontLeft =
      new SwerveModuleOffboard(
        SwerveConstants.kFrontLeftDriveMotorPort,
        SwerveConstants.kFrontLeftTurningMotorPort,
        SwerveConstants.kFrontLeftMagEncoderPort,
        offset_FL);

    m_rearLeft =
      new SwerveModuleOffboard(
        SwerveConstants.kRearLeftDriveMotorPort,
        SwerveConstants.kRearLeftTurningMotorPort,
        SwerveConstants.kRearLeftMagEncoderPort,
        offset_RL);

    m_frontRight =
      new SwerveModuleOffboard(
        SwerveConstants.kFrontRightDriveMotorPort,
        SwerveConstants.kFrontRightTurningMotorPort,
        SwerveConstants.kFrontRightMagEncoderPort,
        offset_FR);

    m_rearRight =
      new SwerveModuleOffboard(
        SwerveConstants.kRearRightDriveMotorPort,
        SwerveConstants.kRearRightTurningMotorPort,
        SwerveConstants.kRearRightMagEncoderPort,
        offset_RR);

    // Odometry class for tracking robot pose
    m_odometry =
        new SwerveDriveOdometry(
            SwerveConstants.kDriveKinematics,
            m_imu.getHeading(),
            new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
            });

            // Create and push Field2d to SmartDashboard.
      m_field = new Field2d();
      SmartDashboard.putData(m_field);

      // Reference: https://www.chiefdelphi.com/t/has-anyone-gotten-pathplanner-integrated-with-the-maxswerve-template/443646

      SmartDashboard.putData(m_field);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_imu.getHeading(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
    
      // Update robot position on Field2d.
    m_field.setRobotPose(getPose());

    m_DriverSpeedScale = Constants.kDriverSpeedLimit;

    pose_publisher.set(getPose());
    swerve_publisher.set(new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState() } ); // :3

    
      // SmartDashboard.putNumber("FR Turn Enc", m_frontRight.getPosition().angle.getDegrees());

      SmartDashboard.putNumber("FL Encoder", m_frontLeft.getPosition().distanceMeters);
      SmartDashboard.putNumber("FR Encoder", m_frontRight.getPosition().distanceMeters);
      SmartDashboard.putNumber("RL Encoder", m_rearLeft.getPosition().distanceMeters);
      SmartDashboard.putNumber("RR Encoder", m_rearRight.getPosition().distanceMeters);


    // Diagnostics
    if (Constants.kDebugLevel >=3) {
      SmartDashboard.putNumber("FL Mag Enc", m_frontLeft.getCanCoder());
      SmartDashboard.putNumber("FR Mag Enc", m_frontRight.getCanCoder());
      SmartDashboard.putNumber("RL Mag Enc", m_rearLeft.getCanCoder());
      SmartDashboard.putNumber("RR Mag Enc", m_rearRight.getCanCoder());

      SmartDashboard.putNumber("FL Angle State", m_frontLeft.getState().angle.getDegrees());
      SmartDashboard.putNumber("FL Angle SparkMax", m_frontLeft.getAngle().getDegrees());
      SmartDashboard.putNumber("FL Angle CanCoder", m_frontLeft.getCanCoder());
      SmartDashboard.putNumber("FL Angle Offset", m_frontLeft.getCanCoder() - m_frontLeft.getAngle().getDegrees());
      SmartDashboard.putNumber("FL Angle Current",m_frontLeft.getTurnCurrent());
      
      SmartDashboard.putNumber("FL Turn Enc", m_frontLeft.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("FR Turn Enc", m_frontRight.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("RL Turn Enc", m_rearLeft.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("RR Turn Enc", m_rearRight.getPosition().angle.getDegrees());
    }
}

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void zeroIMU() {
    m_imu.zeroHeading();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_imu.getHeading(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    //Square inputs
    // xSpeed=Math.signum(xSpeed)* xSpeed*xSpeed;
    // ySpeed=Math.signum(ySpeed)* ySpeed*ySpeed;
    // rot=Math.signum(rot)* rot*rot;

    // Apply joystick deadband
    // xSpeed = MathUtil.applyDeadband(xSpeed, OIConstants.kDeadband, 1.0);
    // ySpeed = MathUtil.applyDeadband(ySpeed, OIConstants.kDeadband, 1.0);
    // rot = MathUtil.applyDeadband(rot, OIConstants.kDeadband, 1.0);

    // Apply speed scaling
    xSpeed = xSpeed * m_DriverSpeedScale;
    ySpeed = ySpeed * m_DriverSpeedScale;
    rot = rot * m_DriverSpeedScale;

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot", rot);

    var swerveModuleStates =
        SwerveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_imu.getHeading()) // Rotation2d.fromDegrees(m_imu.getYaw()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[SwerveConstants.kSwerveFL_enum]);
    m_frontRight.setDesiredState(swerveModuleStates[SwerveConstants.kSwerveFR_enum]);
    m_rearLeft.setDesiredState(swerveModuleStates[SwerveConstants.kSwerveRL_enum]);
    m_rearRight.setDesiredState(swerveModuleStates[SwerveConstants.kSwerveRR_enum]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[SwerveConstants.kSwerveFL_enum]);
    m_frontRight.setDesiredState(desiredStates[SwerveConstants.kSwerveFR_enum]);
    m_rearLeft.setDesiredState(desiredStates[SwerveConstants.kSwerveRL_enum]);
    m_rearRight.setDesiredState(desiredStates[SwerveConstants.kSwerveRR_enum]);
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    this.drive(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,false);
    SmartDashboard.putNumber("DriveVelX", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("DriveVelY", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("DriveRotZ", speeds.omegaRadiansPerSecond);
  }
  
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return SwerveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),
                                                           m_frontRight.getState(),
                                                           m_rearLeft.getState(),
                                                           m_rearRight.getState());
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /* Sets how fast the human driver can drive */
  public void setMaxOutput(double val) {
    m_DriverSpeedScale = val;
  }

  public void toggleMaxOutput() {
    if (m_DriverSpeedScale == 1.0){
      m_DriverSpeedScale = Constants.kDriverSpeedLimit;
    } else {
      m_DriverSpeedScale = 1.0;
    }
  }

}