// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class SwerveModule {

private CANSparkMax driveMotor;
private CANSparkMax rotateMotor;

private SparkMaxPIDController driveController;
private SparkMaxPIDController rotateController;

private RelativeEncoder driveEncoder;
private RelativeEncoder rotateEncoder;

private AnalogInput absoluteEncoder;
private boolean angleInverted;

public SwerveModule(int driveID, int rotateID){
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    rotateMotor = new CANSparkMax(rotateID, MotorType.kBrushless);

    driveMotor.setIdleMode(IdleMode.kBrake);
    rotateMotor.setIdleMode(IdleMode.kBrake);

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPositionConversionFactor(DriveConstants.DRIVE_POSITION_CONVERSION);
    driveEncoder.setVelocityConversionFactor(DriveConstants.DRIVE_VELOCITY_CONVERSION);

    rotateEncoder = rotateMotor.getEncoder();
    rotateEncoder.setPositionConversionFactor(DriveConstants.ROTATE_POSITION_CONVERSION);
    rotateEncoder.setVelocityConversionFactor(DriveConstants.ROTATE_VELOCITY_CONVERSION);

    absoluteEncoder = new AnalogInput(DriveConstants.CANENCODER_ID);

    driveController = driveMotor.getPIDController();
    rotateController = rotateMotor.getPIDController();

    driveController.setP(DriveConstants.P_VALUE);
    driveController.setD(DriveConstants.D_VALUE);
    driveController.setFF(DriveConstants.FF_VALUE);

    rotateController.setP(DriveConstants.P_VALUE);
    rotateController.setD(DriveConstants.D_VALUE);
    rotateController.setFF(DriveConstants.FF_VALUE);

    rotateController.setPositionPIDWrappingMaxInput(Math.PI);
    rotateController.setPositionPIDWrappingMinInput(-Math.PI);
    rotateController.getPositionPIDWrappingEnabled();
    
    resetEncoders();
}

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getRotatePosition() {
        return rotateEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }
    
    public double getRotateVelocity() {
        return rotateEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2 * Math.PI;
        angle -= DriveConstants.ENCODER_OFFSET;
        return angle * (angleInverted ? -1 : 1);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        rotateEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getRotatePosition())); 
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, new Rotation2d(getRotatePosition()));
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.MAX_SPEED);
    }

    public void stop(){
        driveMotor.set(0);
        rotateMotor.set(0);
    }
}
