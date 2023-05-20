// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;

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
        public static final double MAX_SPEED = 0.5;

        //PID Values for swerve modules
        public static final double P_VALUE = 0.001;
        public static final double FF_VALUE = 0.001;
        public static final double D_VALUE = 0.001;
        //*TO DO: Reinstate separate PID vals for drive and rotate motors */

        //in meters
        public static final double TRACK_WIDTH = 1;
        public static final double BASE_LENGTH = 1.5;

        public static final double GEAR_RATIO = 6.75;
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.5);

        //Calculates the distance from the center using the pythagorean theorem
        public final static double DIST_FROM_CENTER = Math.sqrt(Math.pow(TRACK_WIDTH, 2) + Math.pow(BASE_LENGTH, 2));

        //Calculates the angle between the x-axis and the vector
        public final static double CENTER_ANGLE = Math.atan((TRACK_WIDTH / 2) / (BASE_LENGTH / 2));


    }
}
