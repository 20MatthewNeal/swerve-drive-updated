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
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class SwerveModule {

private CANSparkMax driveMotor;
private CANSparkMax rotateMotor;

private PIDController driveController;
private PIDController rotateController;

private RelativeEncoder driveEncoder;
private RelativeEncoder rotateEncoder;

private AnalogInput absoluteEncoder;
private boolean angleInverted;

private Rotation2d lastAngle;

public static EncoderOffsets currOffset = EncoderOffsets.DEFAULT;
public static double offset;
private Class<EncoderOffsets> type;

public SwerveModule(int driveID, int rotateID, int magEncoderPort, boolean invertRotate, boolean invertDrive){
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    rotateMotor = new CANSparkMax(rotateID, MotorType.kBrushless);

    driveMotor.setIdleMode(IdleMode.kBrake);
    rotateMotor.setIdleMode(IdleMode.kBrake);

    if(invertRotate){
        rotateMotor.setInverted(true);
    }

    if(invertDrive){
        driveMotor.setInverted(true);
    }

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPositionConversionFactor(DriveConstants.DRIVE_POSITION_CONVERSION);
    driveEncoder.setVelocityConversionFactor(DriveConstants.DRIVE_VELOCITY_CONVERSION);

    rotateEncoder = rotateMotor.getEncoder();
    rotateEncoder.setPositionConversionFactor(DriveConstants.ROTATE_POSITION_CONVERSION);
    rotateEncoder.setVelocityConversionFactor(DriveConstants.ROTATE_VELOCITY_CONVERSION);

    absoluteEncoder = new AnalogInput(magEncoderPort);

    driveController = new PIDController(DriveConstants.DRIVE_P_VALUE, 0, DriveConstants.DRIVE_D_VALUE);
    rotateController = new PIDController(DriveConstants.ROTATE_P_VALUE, DriveConstants.ROTATE_I_VALUE, 0);

    rotateController.enableContinuousInput(-Math.PI, Math.PI);
    
    lastAngle = getState().angle;

    resetEncoder(EncoderOffsets.FRONT_LEFT);
    resetEncoder(EncoderOffsets.FRONT_RIGHT);
    resetEncoder(EncoderOffsets.BACK_LEFT);
    resetEncoder(EncoderOffsets.BACK_RIGHT);
}

    public enum EncoderOffsets{
        FRONT_LEFT(DriveConstants.FRONT_LEFT_ENCODER_OFFSET),
        FRONT_RIGHT(DriveConstants.FRONT_RIGHT_ENCODER_OFFSET),
        BACK_LEFT(DriveConstants.BACK_LEFT_ENCODER_OFFSET),
        BACK_RIGHT(DriveConstants.BACK_RIGHT_ENCODER_OFFSET),
        DEFAULT(0.0);

        public double encoderVals;

        private EncoderOffsets(double offset){
            this.encoderVals = offset;
            }
        }

    public void toggle(EncoderOffsets e){
        switch(e){
            case FRONT_LEFT:
                currOffset = EncoderOffsets.FRONT_LEFT;
            case FRONT_RIGHT:
                currOffset = EncoderOffsets.FRONT_RIGHT;
            case BACK_LEFT:
                currOffset = EncoderOffsets.BACK_LEFT;
            case BACK_RIGHT:
                currOffset = EncoderOffsets.BACK_RIGHT;
            default: 
                currOffset = EncoderOffsets.DEFAULT;
        }
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
     * @return meters
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

    public double getAbsoluteEncoderRad(EncoderOffsets encoder) {
        toggle(encoder);
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2 * Math.PI;
        angle -= currOffset.encoderVals;
        return angle * (angleInverted ? -1 : 1);
    }

    public void resetEncoder(EncoderOffsets encoder) {
        toggle(encoder);
        driveEncoder.setPosition(0);
        rotateEncoder.setPosition(getAbsoluteEncoderRad(encoder));
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
