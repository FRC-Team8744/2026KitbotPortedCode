// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.ShooterIntake;
import frc.robot.subsystems.drive.DriveSubsystem;
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
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterIntake m_shooterIntake = new ShooterIntake();
  private final Indexer m_indexer = new Indexer();


  //TODO: Once waitForSpeed is fix, fix these
  private final Command intake = 
    m_shooterIntake.intake()
      .alongWith(m_indexer.in());
  private final Command shoot = 
    m_shooterIntake.shoot()
       .alongWith(m_shooterIntake.waitForSpeed()
        .andThen(m_indexer.in()));
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
    SmartDashboard.putData(m_shooterIntake);
    SmartDashboard.putData(m_indexer);

    // Configure the button bindings
    configureButtonBindings();

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
}