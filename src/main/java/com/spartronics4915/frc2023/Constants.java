// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

    
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }
    public static class ArmConstants{
        public static class PIDConstants {
            public final double kP;
            public final double kI;
            public final double kD;
            public PIDConstants(double kP, double kI, double kD) {
                super();
                this.kP = kP;
                this.kI = kI;
                this.kD = kD;
            } 
        }
        public static final PIDConstants kShoulderPID = new PIDConstants(0.01, 0, 0);
        public static final PIDConstants kWristPID = new PIDConstants(0.01, 0, 0);
        public static final int kShoulderMotorId = 0;
        public static final int kWristMotorId = 1;
    }
}
