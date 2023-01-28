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
        public static class AbsoluteEncoderConstants{
            public final int channel; 
            public final double angleOffset;
            public AbsoluteEncoderConstants(int channel, double angleOffset) {
                this.channel = channel;
                this.angleOffset = angleOffset;
            }
        }
        public static class MotorSetupConstants{
            public static final int kShoulderMotorId = 0; //PlaceHolder Value
            public static final int kWristMotorId = 1; //PlaceHolder Value
            public static final PIDConstants kShoulderPID = new PIDConstants(0.01, 0, 0); //PlaceHolder Value
            public static final PIDConstants kWristPID = new PIDConstants(0.01, 0, 0); //PlaceHolder Value
            public static final AbsoluteEncoderConstants kShoulderAbsEncoder = new AbsoluteEncoderConstants(0, 0.0); //PlaceHolder Value
            public static final AbsoluteEncoderConstants kWristAbsEncoder = new AbsoluteEncoderConstants(0, 0.0); //PlaceHolder Value
        }
        public static class LinearActuatorConstants{
            public static final int kLinearActuatorMotorId = 0; //PlaceHolder Value
            public static final PIDConstants kLinearActuatorPID = new PIDConstants(0.01, 0, 0); //PlaceHolder Value
        }
        public static class ClawConstants{
            public static final PIDConstants kClawMotorPID = new PIDConstants(0, 0, 0); //PlaceHolder Value
            public static final int klimitSwitchID = 0; //PlaceHolder Value
            public static final int kClawMotorID = 0; //PlaceHolder Value
            public static final double kInSpeed = 0.25; //PlaceHolder Value
            public static final double kOutSpeed = 0.25; //PlaceHolder Value, already negative in code
            public static final double kGrabTimerLength = 1; //seconds
            public static final double kReleaseTimerLength = 1; //seconds
        }
        public static class ArmDesiredStates{
            public static class RelativePos{
                //insert relative translations 2ds here for movments like going .5 feet down or .5 feet back
            }
            public static class AbsolutePos{
                //insert absolute translations 2ds here for movments like going to above the 1st layer of cones
            }
        }
    }
    
}
