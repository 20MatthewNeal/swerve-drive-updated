// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class SwerveModule {

private CANSparkMax driveMotor;
private CANSparkMax rotateMotor;

private RelativeEncoder driveEncoder;
private RelativeEncoder rotateEncoder;

private PIDController rotateController;

private AnalogInput absoluteEncoder;
private boolean absoluteEncoderReverse;

private double encoderOffset;

private Rotation2d lastAngle;

public static double offset;

public SwerveModule(int driveID, int rotateID, int magEncoderPort, boolean invertRotate, boolean invertDrive, double encoderOffset){
    
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    rotateMotor = new CANSparkMax(rotateID, MotorType.kBrushless);

    driveMotor.setIdleMode(IdleMode.kBrake);
    rotateMotor.setIdleMode(IdleMode.kBrake);
    rotateMotor.setInverted(invertRotate);
    driveMotor.setInverted(invertDrive);

    this.encoderOffset = encoderOffset;

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPositionConversionFactor(DriveConstants.DRIVE_POSITION_CONVERSION);
    driveEncoder.setVelocityConversionFactor(DriveConstants.DRIVE_VELOCITY_CONVERSION);

    rotateEncoder = rotateMotor.getEncoder();
    rotateEncoder.setPositionConversionFactor(DriveConstants.ROTATE_POSITION_CONVERSION);
    rotateEncoder.setVelocityConversionFactor(DriveConstants.ROTATE_VELOCITY_CONVERSION);

    absoluteEncoder = new AnalogInput(magEncoderPort);

    rotateController = new PIDController(DriveConstants.ROTATE_P_VALUE, DriveConstants.ROTATE_I_VALUE, DriveConstants.ROTATE_D_VALUE);
    rotateController.enableContinuousInput(-Math.PI, Math.PI);
    
    // lastAngle = getState().angle;
    resetEncoder();
}

    /**
     * @return meters
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * @return angle (in radians)
     */
    public double getRotatePosition() {
        return rotateEncoder.getPosition();
    }

    /**
     * @return meters / sec
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }
    
    /**
     * @return meters
     */
    public double getRotateVelocity() {
        return rotateEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2 * Math.PI;
        angle -= encoderOffset;
        return angle * (absoluteEncoderReverse ? -1.0 : 1.0);
    }

    public void resetEncoder() {
        driveEncoder.setPosition(0);
        rotateEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getRotatePosition())); 
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.MAX_DRIVE_SPEED);
        rotateMotor.set(rotateController.calculate(getRotatePosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void setAngle(SwerveModuleState desiredAngle) {
        Rotation2d angle = (Math.abs(desiredAngle.speedMetersPerSecond) <= DriveConstants.MAX_DRIVE_SPEED * 0.01) ? lastAngle : desiredAngle.angle;
        rotateController.setSetpoint(angle.getRadians());
        lastAngle = angle;
    }

    public void stop() {
        driveMotor.set(0);
        rotateMotor.set(0);
    }
}
