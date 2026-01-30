// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMUConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Multi_IMU extends SubsystemBase {
  public static final int PIGEON_CAN_ID = 15;

  // The imu sensors
  private PigeonIMU m_pigeon;

  public Multi_IMU() {
    // Attempt to communicate with new sensor (may not exist)
    m_pigeon = new PigeonIMU(PIGEON_CAN_ID);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pigeon2 GyroZ", m_pigeon.getYaw());
  }

  /**
   * Zeros the heading of the robot.
   */
  public void zeroHeading() {
    m_pigeon.setYaw(0);
  }

  /**
   * Returns the heading of the robot as Rotation2D
   *
   * @return the robot's heading as Rotation2D
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  /**
   * Returns human readable heading of the robot.
   *
   * @return The robot's current heading in degrees.
   */
  public double getHeadingDegrees() {
    return m_pigeon.getYaw();
  }

}
