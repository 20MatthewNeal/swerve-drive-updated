// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static final class DriveConstants{
        public static final int DRIVE_JOYSTICK_ID = 0;

        public static final int FRONT_LEFT_ENCODER_ID = 1;
        public static final int FRONT_RIGHT_ENCODER_ID = 3;
        public static final int BACK_LEFT_ENCODER_ID = 0;
        public static final int BACK_RIGHT_ENCODER_ID = 2;

        public static final double MAX_DRIVE_SPEED = 14.5;
        public static final double MAX_ROTATE_SPEED = 4 * Math.PI;

        // PID Values for swerve modules
        public static final double ROTATE_P_VALUE = 0.5;
        public static final double ROTATE_I_VALUE = 0.0;
        public static final double ROTATE_D_VALUE = 0.0;

        // In meters
        public static final double TRACK_WIDTH = Units.inchesToMeters(29);
        public static final double BASE_LENGTH = Units.inchesToMeters(29);

        // Encoder Offsets
        public static final double FRONT_LEFT_ENCODER_OFFSET = 4.927;
        public static final double FRONT_RIGHT_ENCODER_OFFSET = 0.13;
        public static final double BACK_LEFT_ENCODER_OFFSET = 0.471; // Fix when reconnected 
        public static final double BACK_RIGHT_ENCODER_OFFSET = 1.099;

        // Drive Properties
        public static final double DRIVE_GEAR_RATIO = 1 / 6.75;
        public static final double DRIVE_POSITION_CONVERSION = DRIVE_GEAR_RATIO * Math.PI * Units.inchesToMeters(4); // Rotations to meters
        public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60;
        
        // Rotate Properties
        public static final double ROTATE_GEAR_RATIO = 1 / 12.75;
        public static final double ROTATE_POSITION_CONVERSION = ROTATE_GEAR_RATIO * Math.PI * 2; // Rotations to radians
        public static final double ROTATE_VELOCITY_CONVERSION = ROTATE_POSITION_CONVERSION / 60;

        // Wheel Properties
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
        public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * (WHEEL_DIAMETER / 2.0);

        //Calculates the distance from the center using the pythagorean theorem
        public final static double DIST_FROM_CENTER = Math.sqrt(Math.pow(TRACK_WIDTH, 2) + Math.pow(BASE_LENGTH, 2));

        //Calculates the angle between the x-axis and the vector
        public final static double CENTER_ANGLE = Math.atan((TRACK_WIDTH / 2) / (BASE_LENGTH / 2));

        public final static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(BASE_LENGTH / 2, -TRACK_WIDTH / 2),
            new Translation2d(BASE_LENGTH / 2, TRACK_WIDTH / 2),
            new Translation2d(-BASE_LENGTH / 2, -TRACK_WIDTH / 2),
            new Translation2d(-BASE_LENGTH / 2, TRACK_WIDTH / 2));
    }
}
