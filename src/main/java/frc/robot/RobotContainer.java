// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.AlignToHub;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.ShooterIntake;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.PhotonVision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

 private final Rotation3d cameraToRobotOffsetRotationLeft = new Rotation3d(Units.degreesToRadians(12.75), Units.degreesToRadians(22.25), Units.degreesToRadians(-56.2));
  private final Rotation3d cameraToRobotOffsetRotationRight = new Rotation3d(Units.degreesToRadians(12.75), Units.degreesToRadians(-22.25), Units.degreesToRadians(-56.2));

  private final Transform3d cameraToRobotOffsetLeft = new Transform3d(Units.inchesToMeters(-22.5*0.39370079), Units.inchesToMeters(-27.5*0.39370079), Units.inchesToMeters(21*0.39370079), cameraToRobotOffsetRotationLeft);
  private final Transform3d cameraToRobotOffsetRight = new Transform3d(Units.inchesToMeters(-22.5*0.39370079), Units.inchesToMeters(27.5*0.39370079), Units.inchesToMeters(21*0.39370079), cameraToRobotOffsetRotationRight);
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();
  // private final PhotonVision.Context photonVisionContext = new PhotonVision.Context(aprilTagFieldLayout, new PhotonVision.CameraWithOffsets("Limelight4.1", cameraToRobotOffset1), new PhotonVision.CameraWithOffsets("Limelight4.2", cameraToRobotOffset2));
  private final PhotonVision.Context photonVisionContext = new PhotonVision.Context(aprilTagFieldLayout, new PhotonVision.CameraWithOffsets("back_left_camera", cameraToRobotOffsetLeft), new PhotonVision.CameraWithOffsets("back_right_camera", cameraToRobotOffsetRight));
  private final PhotonVision m_visionPV = new PhotonVision(photonVisionContext);
  
  private final AlignToHub m_alignToHub = new AlignToHub();


  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_visionPV, m_alignToHub);
  private final ShooterIntake m_shooterIntake = new ShooterIntake();
  private final Indexer m_indexer = new Indexer();

  private final SendableChooser<Command> m_autoChooser;

  //TODO: Once waitForSpeed is fix, fix these
  private final Command intake = 
    m_shooterIntake.intake()
      .alongWith(m_indexer.in());
  private final Command shoot = 
    m_shooterIntake.shoot()
       .alongWith(m_shooterIntake.waitForSpeed()
        .andThen(m_indexer.out()));
  private final Command outtake = 
    m_shooterIntake.outtake()
      .alongWith(m_indexer.out());
  private final Command stopAll =
    m_indexer.stop()
      .alongWith(m_shooterIntake.stop());
  // private final Command intakeShoot = 
  //   m_shooterIntake.shoot()
  //      .alongWith(m_indexer.in())
  //       .alongWith(m_shooterIntake.waitForSpeed()
  //          .andThen(m_indexer.out()));

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // XboxController m_codriverController = new XboxController(OIConstants.kCodriverControllerPort);
  CommandXboxController m_driver = new CommandXboxController(OIConstants.kDriverControllerPort);

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(5);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", m_autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                   // -m_driverController.getLeftY() *SwerveConstants.kMaxSpeedTeleop,
                   //-m_driverController.getLeftX() *SwerveConstants.kMaxSpeedTeleop,
                   // -m_driverController.getRightX() *ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND,
                     m_xspeedLimiter.calculate( -m_driverController.getLeftY() )*SwerveConstants.kMaxSpeedTeleop,
                     m_yspeedLimiter.calculate( -m_driverController.getLeftX() )*SwerveConstants.kMaxSpeedTeleop,
                     m_rotLimiter.calculate( -m_driverController.getRightX() )*ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND,
                    true),
            m_robotDrive));
    }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  
  private void configureButtonBindings() {
    m_driver.leftBumper().whileTrue(intake);
    m_driver.rightBumper().whileTrue(shoot);
    m_driver.a().whileTrue(outtake);
    m_driver.back().whileTrue(Commands.runOnce(() -> m_robotDrive.m_imu.zeroHeading()));
    // m_driver.y().whileTrue(stopAll);
   // m_driver.rightTrigger().whileTrue(intakeShoot);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}