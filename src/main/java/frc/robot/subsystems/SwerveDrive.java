// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.SwerveModule;

public class SwerveDrive extends SubsystemBase {
  
  private SwerveModule frontLeft;
  private SwerveModule frontRight;
  private SwerveModule backLeft;
  private SwerveModule backRight;

  private SwerveDriveKinematics swerveDrive;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    frontLeft = new SwerveModule(1, 5);
    frontRight = new SwerveModule(2, 6);
    backLeft = new SwerveModule(3, 7);
    backRight = new SwerveModule(4, 7);

    swerveDrive = new SwerveDriveKinematics(new Translation2d(DriveConstants.DIST_FROM_CENTER, DriveConstants.CENTER_ANGLE));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
