// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private SwerveDriveOdometry swerveOdometry;

  private SwerveModulePosition[] positions;
  private SwerveModuleState[] states;

  private boolean fieldOriented;

  private AHRS gyro;

  private SendableBuilder sendableBuilder;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    
    frontLeft = new SwerveModule(1, 5, DriveConstants.FRONT_LEFT_ENCODER_ID, false, false, DriveConstants.FRONT_LEFT_ENCODER_OFFSET);
    frontRight = new SwerveModule(2, 6, DriveConstants.FRONT_RIGHT_ENCODER_ID, false, false, DriveConstants.FRONT_RIGHT_ENCODER_OFFSET);
    backLeft = new SwerveModule(3, 7, DriveConstants.BACK_LEFT_ENCODER_ID, false, false, DriveConstants.BACK_LEFT_ENCODER_OFFSET);
    backRight = new SwerveModule(4, 8, DriveConstants.BACK_RIGHT_ENCODER_ID, false, false, DriveConstants.BACK_RIGHT_ENCODER_OFFSET);

    gyro = new AHRS(SPI.Port.kMXP);
    
    positions = new SwerveModulePosition[] {
    getModulePosition("Front Left"),
    getModulePosition("Front Right"), 
    getModulePosition("Back Left"), 
    getModulePosition("Back Right")};
    
    swerveOdometry = new SwerveDriveOdometry(DriveConstants.swerveKinematics, gyro.getRotation2d(), positions, new Pose2d(0,0, new Rotation2d(getHeading())));

    fieldOriented = false;

    //Resets the angle of the gyro at the beginning of run
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {}
    }).start();
  }
  
  
  public SwerveModulePosition getModulePosition(String module) {

    SwerveModulePosition position;

    if(module.equals("Front Left")){
      position = new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getRotatePosition()));
      return position;
    }
    else if(module.equals("Front Right")) {
      position = new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getRotatePosition()));
      return position;
    }
    else if(module.equals("Back Left")) {
      position = new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getRotatePosition()));
      return position;
    }
    else if(module.equals("Back Right")) {
      position = new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getRotatePosition()));
      return position;
    }
    
    return null;
    }


  public boolean getFieldOriented() {
    return fieldOriented;
  }

  public void setFieldOriented() {
    fieldOriented = !fieldOriented;
  }

  public void setFieldOriented(boolean condition) {
    fieldOriented = condition;
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading()); 
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    swerveOdometry.resetPosition(getRotation2d(), positions, pose);
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_DRIVE_SPEED); // Ensures that each module stays within its range of velocity
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
    positions[0] = new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getRotatePosition()));
    positions[1] = new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getRotatePosition()));
    positions[2] = new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getRotatePosition()));
    positions[3] = new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getRotatePosition()));
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    states[0] = new SwerveModuleState(frontLeft.getDriveVelocity(), new Rotation2d(frontLeft.getRotatePosition()));
    states[1] = new SwerveModuleState(frontRight.getDriveVelocity(), new Rotation2d(frontRight.getRotatePosition()));
    states[2] = new SwerveModuleState(backLeft.getDriveVelocity(), new Rotation2d(backLeft.getRotatePosition()));
    states[3] = new SwerveModuleState(backRight.getDriveVelocity(), new Rotation2d(backRight.getRotatePosition()));
    return states;
  }


  @Override
  public void initSendable(SendableBuilder sendableBuilder) {
    sendableBuilder.setSmartDashboardType("Encoder Values");
    sendableBuilder.addDoubleProperty("Front Left", () -> frontLeft.getRotatePosition(), null);
    sendableBuilder.addDoubleProperty("Front Right", () -> frontRight.getRotatePosition(), null);
    sendableBuilder.addDoubleProperty("Back Left", () -> backLeft.getRotatePosition(), null);
    sendableBuilder.addDoubleProperty("Back Right", () -> backRight.getRotatePosition(), null);
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getRotation2d(), positions);
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
  }
}
