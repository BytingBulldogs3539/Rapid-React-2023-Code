// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import org.bytingbulldogs.bulldoglibrary.INIConfiguration.BBConstants;

/** Add your docs here. */
public class IntakeConstants extends BBConstants {
    public IntakeConstants() {
        super("/home/lvuser/IntakeConstants.ini", true);
        save();
    }

    public static boolean invertDirection = true;
    public static int currentLimit = 40;



}
