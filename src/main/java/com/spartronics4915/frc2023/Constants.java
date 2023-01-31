// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023;

import java.util.function.Function;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * 
 * <p>UNITS: All distances, positions, displacements, etc. should be defined in meters. All
 * angles should be measured in radians. All times and time intervals should be measured in seconds. 
 * 
 * <p>ROBOT ORIENTED: Robot forward is positive x, robot left is positive y, CCW rotation is positive, 
 * and the angle measure 0 is directly forward.
 * 
 * <p>FIELD ORIENTED: Away from alliance wall is positive x, left relative to driver station is y, CCW 
 * rotation is positive, and the angle measure 0 is directly away from the alliance wall.
 */

 
public final class Constants {
	public static final class Trajectory {
		public static final double kLinearP = 1.0;
		public static final double kThetaP = 1.0;
	}

    public static final class Swerve {
        public static final class Drive {
            public static final double kP = 0.0; // placeholder
            public static final double kI = 0.0; // placeholder
            public static final double kD = 0.0; // placeholder
            public static final double kFF = 0.0; // placeholder
            
            public static final double kS = 0.0; // placeholder
            public static final double kV = 0.0; // placeholder
            public static final double kA = 0.0; // placeholder

            public static final int kContinuousCurrentLimit = 30;

            public static final double kGearRatio = 6.75 / 1.0;
            public static final double kVelocityConversionFactor = ((kWheelDiameter * Math.PI) / kGearRatio) / 60.0;
            public static final double kPositionConversionFactor = ((kWheelDiameter * Math.PI) / kGearRatio);
        }

        public static final class Angle {
            public static final double kP = 1.0;
            public static final double kI = 0.0;
            public static final double kD = 0.5;
            public static final double kFF = 0;

            public static final int kContinuousCurrentLimit = 15;

            public static final double kGearRatio = 150.0 / 7.0;
            public static final double kPositionConversionFactor = (2 * Math.PI) / (kGearRatio);
        }

        public static final int kPigeonID = 9;

        public static final double kPigeonMountPoseYaw = 0;
        public static final double kPigeonMountPosePitch = 0;
        public static final double kPigeonMountPoseRoll = 180;
        
        public static final double kTrackWidth = Units.inchesToMeters(18.75);
        public static final double kWheelBase = Units.inchesToMeters(23.75);
        public static final double kChassisRadius = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
		public static final Pose2d kInitialPose = new Pose2d();

        public static final double kMaxSpeed = Units.feetToMeters(14.5);
        public static final double kMaxAngularSpeed = kMaxSpeed / kChassisRadius;
        public static final double kMaxAcceleration = Units.feetToMeters(14.5); // TODO: get an actual value because this should be higher

        public static final double kSlowModeSpeedMultiplier = 0.3;
        public static final double kSlowModeAngularSpeedMultiplier = 0.3;

        public static final double kWheelDiameter = Units.inchesToMeters(4.0);
        
        public static final Function<Integer, CANSparkMax> kMotorConstructor = (Integer ID) -> { return new CANSparkMax(ID, MotorType.kBrushless); };
        
        public static final boolean kDriveMotorsAreInverted = true;
        public static final boolean kAngleMotorsAreInverted = false;

        public static final IdleMode kDriveIdleMode = IdleMode.kBrake;
        public static final IdleMode kAngleIdleMode = IdleMode.kBrake;

        public static final double kVoltageCompensation = 12.0;

        public static final class Module0 {
            public static final int kDriveMotorID = 5;
            public static final int kAngleMotorID = 6;
            public static final int kEncoderID = 13;
            public static final double kRawAngleOffsetDegrees = -96.855;
            public static final double kRawAngleOffsetRotations = kRawAngleOffsetDegrees / 360;
			public static final double kAngleOffset = Math.PI * 2 * kRawAngleOffsetRotations;
            public static final SwerveModuleConstants kConstants = 
                new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kEncoderID, kAngleOffset, kRawAngleOffsetRotations);
        }

        public static final class Module1 {
            public static final int kDriveMotorID = 3;
            public static final int kAngleMotorID = 4;
            public static final int kEncoderID = 12;
            public static final double kRawAngleOffsetDegrees = -168.486;
            public static final double kRawAngleOffsetRotations = kRawAngleOffsetDegrees / 360;
			public static final double kAngleOffset = Math.PI * 2 * kRawAngleOffsetRotations;
            public static final SwerveModuleConstants kConstants = 
                new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kEncoderID, kAngleOffset, kRawAngleOffsetRotations);
        }

        public static final class Module2 {
            public static final int kDriveMotorID = 7;
            public static final int kAngleMotorID = 8;
            public static final int kEncoderID = 14;
            public static final double kRawAngleOffsetDegrees = -15.820;
            public static final double kRawAngleOffsetRotations = kRawAngleOffsetDegrees / 360;
			public static final double kAngleOffset = Math.PI * 2 * kRawAngleOffsetRotations;
            public static final SwerveModuleConstants kConstants = 
                new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kEncoderID, kAngleOffset, kRawAngleOffsetRotations);
        }

        public static final class Module3 {
            public static final int kDriveMotorID = 1;
            public static final int kAngleMotorID = 2;
            public static final int kEncoderID = 11;
            public static final double kRawAngleOffsetDegrees = -118.916;
            public static final double kRawAngleOffsetRotations = kRawAngleOffsetDegrees / 360;
			public static final double kAngleOffset = Math.PI * 2 * kRawAngleOffsetRotations;
            public static final SwerveModuleConstants kConstants = 
                new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kEncoderID, kAngleOffset, kRawAngleOffsetRotations);
        }

        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // front right, module 1
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // front left, module 0
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0), // back right, module 3
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0) // back left, module 2
        );
        
        public static class SwerveModuleConstants {
            public final int driveMotorID;
            public final int angleMotorID;
            public final int encoderID;
            public final double angleOffset;
            public final double absoluteOffset;

            public SwerveModuleConstants(int d, int a, int e, double o, double absoluteOffset_) {
                driveMotorID = d;
                angleMotorID = a;
                encoderID = e;
                angleOffset = o;
                absoluteOffset = absoluteOffset_;
            }
        }
    }

    public static final class Camera {
        public static final String kFrontCameraName = "frontcamera";

        public static final Pose2d[] kTagPoses = new Pose2d[] {
            null,
            new Pose2d()
        };

        public static final Transform2d kFrontCameraToRobot = new Transform2d(
            new Pose2d(0, 0, new Rotation2d(0)), // camera
            new Pose2d(0, 0, new Rotation2d(0)) // robot (0)
        );
    }

    public static final class OI {
        public static final int kControllerID = 2;

        public static final int kToggleFieldRelativeButton = 1;
        public static final int kResetYawButton = 8;
        public static final int kResetOdometryButton = 3;
        public static final int kSlowModeAxis = 3;
        public static final int kAimButton = 4;

        public static final double kStickDeadband = 0.08;
        public static final double kTriggerDeadband = 0.08;
        public static final double kResponseCurveExponent = 5.0 / 3.0;
    }

    public static final class FieldConstants {
        static final double length = Units.feetToMeters(54);
        static final double width = Units.feetToMeters(27);
    }

    public static final class VisionConstants {
        static final Transform3d robotToCam =
                new Transform3d(
                        new Translation3d(0.5, 0.0, 0.5),
                        new Rotation3d(
                                0, 0,
                                0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        static final String cameraName = "Arducam_OV9281_USB_Camera";
    }
}