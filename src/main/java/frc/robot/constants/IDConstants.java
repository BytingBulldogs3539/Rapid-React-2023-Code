// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import org.bytingbulldogs.bulldoglibrary.INIConfiguration.BBConstants;

/** Add your docs here. */
public class IDConstants extends BBConstants {
    public IDConstants() {
        super("/home/lvuser/IDConstants.ini", true);
        save();
    }

    public static int FLDriveID = 2;
    public static int FRDriveID = 18;
    public static int BLDriveID = 4;
    public static int BRDriveID = 42;
    public static int FLSteeringID = 1;
    public static int FRSteeringID = 17;
    public static int BRSteeringID = 41;
    public static int BLSteeringID = 3;
    public static int FLCanEncoderID = 30;
    public static int FRCanEncoderID = 33;
    public static int BLCanEncoderID = 31;
    public static int BRCanEncoderID = 32;


    public static int intakeMotorID = 0;


    public static int mainWheelsID = 11;
    public static int backWheelsID = 5;
    public static int kickerWheelsID = 12;
    public static int digitalInputID = 0;

    public static int compressorID = 25;
    public static int intakeSolenoidIn = 3;
    public static int intakeSolenoidOut = 2;

}
