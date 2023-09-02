// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import org.bytingbulldogs.bulldoglibrary.INIConfiguration.BBConstants;

/** Add your docs here. */
public class ShooterConstants extends BBConstants {
    public ShooterConstants() {
		super("/home/lvuser/ShooterConstants.ini", true);
		save();
	}

        public static double mainWheelsKP = 0.1;
        public static double mainWheelsKI = 0;
        public static double mainWheelsKD = 5;
        public static double mainWheelsKF = 0.045;
        public static double mainWheelsGearRatio = 1;
        public static int mainWheelsCurrentLimit = 40;
        public static boolean mainWheelsInvert = true;


        public static double backWheelsKP = 0.005;
        public static double backWheelsKI = 0;
        public static double backWheelsKD = 0;
        public static double backWheelsKF = 0.045;
        public static double backWheelsGearRatio = 1;
        public static int backWheelsCurrentLimit = 40;
        public static boolean backWheelsInvert = true;

        public static double kickerWheelsKP = 0.005;
        public static double kickerWheelsKI = 0;
        public static double kickerWheelsKD = 0;
        public static double kickerWheelsKF = 0.045;
        public static double kickerWheelsGearRatio = 2;
        public static int kickerWheelsCurrentLimit = 40;
        public static boolean kickerWheelsInvert = true;

        public static boolean invertSensor = true;
}
