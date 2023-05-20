// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class SwerveModule {

private CANSparkMax driveMotor;
private CANSparkMax rotateMotor;

private SparkMaxPIDController driveController;
private SparkMaxPIDController rotateController;

private RelativeEncoder driveEncoder;
private RelativeEncoder rotateEncoder;

public SwerveModule(int driveID, int rotateID){
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    rotateMotor = new CANSparkMax(rotateID, MotorType.kBrushless);

    driveController = driveMotor.getPIDController();
    rotateController = rotateMotor.getPIDController();

    driveController.setP(DriveConstants.P_VALUE);
    driveController.setD(DriveConstants.D_VALUE);
    driveController.setFF(DriveConstants.FF_VALUE);

    rotateController.setP(DriveConstants.P_VALUE);
    rotateController.setD(DriveConstants.D_VALUE);
    rotateController.setFF(DriveConstants.FF_VALUE);

    driveMotor.setIdleMode(IdleMode.kBrake);
    rotateMotor.setIdleMode(IdleMode.kBrake);

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPositionConversionFactor(Constants.DriveConstants.GEAR_RATIO * 2 * Math.PI);
    driveEncoder.setVelocityConversionFactor(Constants.DriveConstants.GEAR_RATIO * 2 * Math.PI);

    rotateEncoder = rotateMotor.getEncoder();

}

public void driveWithVelocity(double velocity){
    driveMotor.set(velocity);
}

public void drive(double velocity){
    driveController.setReference(velocity, ControlType.kVelocity);
}

}
