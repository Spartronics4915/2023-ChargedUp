// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023;

import java.util.HashMap;
import java.util.function.Function;
import java.util.function.IntFunction;

import com.ctre.phoenix.sensors.BasePigeon;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.spartronics4915.frc2023.subsystems.SwerveModule.AbsoluteAnalogEncoder;
import com.spartronics4915.frc2023.subsystems.SwerveModule.AbsoluteCANCoder;
import com.spartronics4915.frc2023.subsystems.SwerveModule.AbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import static com.spartronics4915.frc2023.subsystems.ArmSubsystem.ArmPosition;

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
		public static final double kLinearP = 0.01;
		public static final double kThetaP = 0.01;
		public static final double kMaxVelocity = 0.5;
		public static final double kMaxAccel = 0.4;
		public static final double kBackUpDistance = 2;
	}

    public static final class Swerve {
        
		public static class ChassisConstants {
			public final double driveGearRatio, angleGearRatio;
			public final double trackWidth, wheelBase;
			public final boolean angleInverted; 
			public final double[] moduleOffsets;
			public final int[] driveMotorIDs, angleMotorIDs, encoderIDs;
			public final IntFunction<BasePigeon> pigeonConstructor;
			public final IntFunction<AbsoluteEncoder> absoluteEncoderConstructor;
			public final int pigeonId;
			public ChassisConstants(
				double driveGearRatio, double angleGearRatio,
				double trackWidth, double wheelBase, boolean angleInverted,
				double[] moduleOffsets,
				int[] driveMotorIDs, int[] angleMotorIDs, int[] encoderIDs,
				IntFunction<AbsoluteEncoder> absoluteEncoderConstructor,
				IntFunction<BasePigeon> pigeonConstructor,
				int pigeonId
			) {
				this.driveGearRatio = driveGearRatio;
				this.angleGearRatio = angleGearRatio;
				this.trackWidth = trackWidth;
				this.wheelBase = wheelBase;
				this.moduleOffsets = moduleOffsets;
				this.pigeonConstructor = pigeonConstructor;
				this.absoluteEncoderConstructor = absoluteEncoderConstructor;
				this.driveMotorIDs = driveMotorIDs;
				this.angleMotorIDs = angleMotorIDs;
				this.encoderIDs = encoderIDs;
				this.pigeonId = pigeonId;
				this.angleInverted = angleInverted;
			}
		}
		public static final ChassisConstants kMk4iChassisConstants = new ChassisConstants(
			6.75 / 1.0, 150.0 / 7.0,
			Units.inchesToMeters(18.75), Units.inchesToMeters(23.75), true,
			new double[]{ 96.328, 167.431, 16.962, 118.652 },
			new int[]{ 5, 3, 7, 9 },
			new int[]{ 6, 4, 8, 10 },
			new int[]{ 13, 12, 14, 11 },
			(int id) -> { return (AbsoluteEncoder)(new AbsoluteCANCoder(id)); },
			(int id) -> { return (BasePigeon)(new Pigeon2(id)); },
			2
		);
		public static final ChassisConstants kMk2ChassisConstants = new ChassisConstants(
			8.33 / 1.0, 18.0 / 1.0,
			0.75, 0.75, false,
			new double[]{ 0.016 * 360, 0.511 * 360, 0.278 * 360, 0.802 * 360 },
			new int[]{ 1, 3, 5, 7 },
			new int[]{ 2, 4, 6, 8 },
			new int[]{ 0, 1, 2, 3 },
			(int id) -> { return (AbsoluteEncoder)(new AbsoluteAnalogEncoder(id)); },
			(int id) -> { return (BasePigeon)(new PigeonIMU(id)); },
			12
		);
		public static final ChassisConstants kChassisConstants = kMk4iChassisConstants;


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

            public static final double kGearRatio =  150.0 / 7.0;
            public static final double kPositionConversionFactor = (2 * Math.PI) / (kGearRatio);

			public static final boolean kInverted = kChassisConstants.angleInverted;
        }

        public static final int kPigeonID = kChassisConstants.pigeonId;
		public static final IntFunction<BasePigeon> kPigeonConstructor = kChassisConstants.pigeonConstructor;

		public static final IntFunction<AbsoluteEncoder> kAbsoluteEncoderConstructor = kChassisConstants.absoluteEncoderConstructor;

        public static final double kPigeonMountPoseYaw = -90;
        public static final double kPigeonMountPosePitch = 0;
        public static final double kPigeonMountPoseRoll = 180;
        
        public static final double kTrackWidth = Units.inchesToMeters(18.75);
        public static final double kWheelBase = Units.inchesToMeters(23.75);
        public static final double kChassisRadius = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
		// public static final Pose2d kInitialPose = new Pose2d();

		public static class InitialPose {
			public String name;
			public Pose2d pose;

			public InitialPose(String name, Pose2d pose) {
				this.name = name;
				this.pose = pose;
			}
		}
		
		public static final InitialPose[] kInitialPoses = {
			new InitialPose("Left", new Pose2d(new Translation2d(), new Rotation2d(Math.PI))),
			new InitialPose("Center", new Pose2d(new Translation2d(), new Rotation2d())),
			new InitialPose("Right", new Pose2d(new Translation2d(), new Rotation2d()))
		};

		public static final int kDefaultInitialPoseIndex = 0;

        public static final double kMaxSpeed = Units.feetToMeters(14.5);
        public static final double kMaxAngularSpeed = kMaxSpeed / kChassisRadius; // ~11.5 rad/s
        public static final double kMaxAcceleration = Units.feetToMeters(14.5); // TODO: get an actual value because this should be higher
        public static final double kMaxAngularAcceleration = kMaxAcceleration / kChassisRadius;

        public static final double kSlowModeSpeedMultiplier = 0.3;
        public static final double kSlowModeAngularSpeedMultiplier = 0.3;

        public static final double kWheelDiameter = Units.inchesToMeters(4.0);
        
        public static final IntFunction<CANSparkMax> kMotorConstructor = (int ID) -> { return new CANSparkMax(ID, MotorType.kBrushless); };
        
        public static final boolean kDriveMotorsAreInverted = true;
        public static final boolean kAngleMotorsAreInverted = false;

        public static final IdleMode kDriveIdleMode = IdleMode.kBrake;
        public static final IdleMode kAngleIdleMode = IdleMode.kBrake;

        public static final double kVoltageCompensation = 12.0;

        public static final class Module0 {
			public static final int kIndex = 0;
            public static final int kDriveMotorID = kChassisConstants.driveMotorIDs[kIndex];
            public static final int kAngleMotorID = kChassisConstants.angleMotorIDs[kIndex];
            public static final int kEncoderID = kChassisConstants.encoderIDs[kIndex];
            public static final double kRawAngleOffsetDegrees = kChassisConstants.moduleOffsets[kIndex];
            public static final double kRawAngleOffsetRotations = kRawAngleOffsetDegrees / 360;
			public static final double kAngleOffset = Math.PI * 2 * kRawAngleOffsetRotations;
            public static final SwerveModuleConstants kConstants = 
                new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kEncoderID, kAngleOffset, kAngleOffset);
        }

        public static final class Module1 {
			public static final int kIndex = 1;
            public static final int kDriveMotorID = kChassisConstants.driveMotorIDs[kIndex];
            public static final int kAngleMotorID = kChassisConstants.angleMotorIDs[kIndex];
            public static final int kEncoderID = kChassisConstants.encoderIDs[kIndex];
            public static final double kRawAngleOffsetDegrees = kChassisConstants.moduleOffsets[kIndex];
            public static final double kRawAngleOffsetRotations = kRawAngleOffsetDegrees / 360;
			public static final double kAngleOffset = Math.PI * 2 * kRawAngleOffsetRotations;
            public static final SwerveModuleConstants kConstants = 
                new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kEncoderID, kAngleOffset, kAngleOffset);
        }

        public static final class Module2 {
			public static final int kIndex = 2;
            public static final int kDriveMotorID = kChassisConstants.driveMotorIDs[kIndex];
            public static final int kAngleMotorID = kChassisConstants.angleMotorIDs[kIndex];
            public static final int kEncoderID = kChassisConstants.encoderIDs[kIndex];
            public static final double kRawAngleOffsetDegrees = kChassisConstants.moduleOffsets[kIndex];
            public static final double kRawAngleOffsetRotations = kRawAngleOffsetDegrees / 360;
			public static final double kAngleOffset = Math.PI * 2 * kRawAngleOffsetRotations;
            public static final SwerveModuleConstants kConstants = 
                new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kEncoderID, kAngleOffset, kAngleOffset);
        }

        public static final class Module3 {
			public static final int kIndex = 3;
            public static final int kDriveMotorID = kChassisConstants.driveMotorIDs[kIndex];
            public static final int kAngleMotorID = kChassisConstants.angleMotorIDs[kIndex];
            public static final int kEncoderID = kChassisConstants.encoderIDs[kIndex];
            public static final double kRawAngleOffsetDegrees = kChassisConstants.moduleOffsets[kIndex];
            public static final double kRawAngleOffsetRotations = kRawAngleOffsetDegrees / 360;
			public static final double kAngleOffset = Math.PI * 2 * kRawAngleOffsetRotations;
            public static final SwerveModuleConstants kConstants = 
                new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kEncoderID, kAngleOffset, kAngleOffset);
        }

        public static final class BalanceConstants {
            public static final class XVelocityPID {
                public static final double kP = 0.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
            }

            // public static final class YVelocityPID {
            //     public static final double kP = 0.0;
            //     public static final double kI = 0.0;
            //     public static final double kD = 0.0;
            // }

            public static final class ThetaPID {
                public static final double kP = 0.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
            }
        }

        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // front left, module 0
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // front right, module 1
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // back left, module 2
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0) // back right, module 3
        );
        
        public static class SwerveModuleConstants {
            public final int driveMotorID;
            public final int angleMotorID;
            public final int encoderID;
            public final double angleOffset;
            public final double absoluteOffset; // radians

            public SwerveModuleConstants(int d, int a, int e, double o, double _absoluteOffsetRadians) {
                driveMotorID = d;
                angleMotorID = a;
                encoderID = e;
                angleOffset = o;
                absoluteOffset = _absoluteOffsetRadians;
            }
        }
    }


    public static final class Arm {
		public static class Auto {
			public static final double kArmStateChangeDuration = 3; // seconds
			public static final double kGrabDuration = 2; // seconds
		}
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

        public static final class ExtenderConstants{
            public static final int kMotorID = 17;
            public static final int kLimitSwitchZeroPort = 9; 
            public static final int kLimitSwitch2Port = -1;
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
        public static final class ArmMotorConstants{
            public final int kMotorID;
            public final double kPositionConversionFactor;
            public final boolean kInverted;
            public final double kP;
            public final double kI;
            public final double kD;
            public final double kFF;
            public final double kSmartMotionMaxAccel;
            public final double kSmartMotionMaxVelocity;
            public final double kSmartMotionMinOutputVelocity;
            public final Rotation2d kZeroOffset;
            public final int kFollowerMotorID;
            public final boolean kInvertMotor;
            public final MotorType kMotorType;

            public ArmMotorConstants(int MotorID, double PositionConversionFactor, boolean Inverted, double P, double I, double D, double FF, 
            double SmartMotionMaxAccel, double SmartMotionMaxVelocity, double SmartMotionMinOutputVelocity, Rotation2d zeroOffset, 
            int followerMotorID, boolean motorInverted, MotorType motorType) {
                super();
                this.kMotorID = MotorID;
                this.kPositionConversionFactor = PositionConversionFactor;
                this.kInverted = Inverted;
                this.kP = P;
                this.kI = I;
                this.kD = D;
                this.kSmartMotionMaxAccel = SmartMotionMaxAccel; 
                this.kSmartMotionMaxVelocity = SmartMotionMaxVelocity;
                this.kSmartMotionMinOutputVelocity = SmartMotionMinOutputVelocity;
                this.kZeroOffset = zeroOffset;
                this.kFollowerMotorID = followerMotorID;
                this.kInvertMotor = motorInverted;
                this.kMotorType = motorType;
                this.kFF = FF;
            }
        }
        public static final IntFunction<CANSparkMax> kNeoConstructor = (int ID) -> { return new CANSparkMax(ID, MotorType.kBrushless); };
        public static final IntFunction<CANSparkMax> k775Constructor = (int ID) -> { return new CANSparkMax(ID, MotorType.kBrushed); };

        public static final ArmMotorConstants kPivotMotorConstants = new ArmMotorConstants(
            15,  //actual value 15
            Math.PI * 2, false,
            0.2, 0, 0, 0.04,
            Math.PI/8, 1, 0,
            Rotation2d.fromDegrees(66), 10, false,
            MotorType.kBrushless
        ); 

        public static final ArmMotorConstants kWristMotorConstants = new ArmMotorConstants(
            19, 
            Math.PI * 2, true,
            0.3, 0, 0, 0.03,
            1, 1, 0, //maybe try lowering max velocity, maybe add limiter variables for smart motion
            Rotation2d.fromDegrees(136),-1, true,
            MotorType.kBrushed
        ); 

        public static final int kPivotFollowerID = 16; //actual value: 16
        
        public static final ArmPositionConstants kRetractedConstants = new ArmPositionConstants(
            0,
            Rotation2d.fromDegrees(-20), //0
            Rotation2d.fromDegrees(30) //0
        );

        public static final ArmPositionConstants kPriorRetracted = new ArmPositionConstants(
            0,
            Rotation2d.fromDegrees(0), //0
            Rotation2d.fromDegrees(20) //0
        );

        public static final ArmPositionConstants kGrabUprightConstants = new ArmPositionConstants(
            0,
            new Rotation2d(Math.PI/4), //45
            new Rotation2d(Math.PI/4) //45
        );

        // public static final ArmPositionConstants kArmLowConstants = new ArmPositionConstants(
        //     0,
        //     Rotation2d.fromDegrees(-20),
        //     Rotation2d.fromDegrees(0)
        // );

        // public static final ArmPositionConstants kArmLevelConstants = new ArmPositionConstants(
        //     0,
        //     Rotation2d.fromDegrees(0),
        //     Rotation2d.fromDegrees(0)
        // );

        // public static final ArmPositionConstants kArmHighConstants = new ArmPositionConstants(
        //     0,
        //     Rotation2d.fromDegrees(20),
        //     Rotation2d.fromDegrees(0)
        // );

        public static final ArmPositionConstants kFloorPositionConstants = new ArmPositionConstants(
            0,
            Rotation2d.fromDegrees(-15),
            Rotation2d.fromDegrees(15)
        );

        public static final ArmPositionConstants kDoubleSubstationConstants = new ArmPositionConstants(
            0,
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(0)
        );
        
        public static final ArmPositionConstants kConeLevel1Constants = new ArmPositionConstants(
            4,
            Rotation2d.fromDegrees(12),
            Rotation2d.fromDegrees(5)//Should be -5,just changed for testing
        );

        public static final ArmPositionConstants kConeLevel2Constants = new ArmPositionConstants(
            14,
            Rotation2d.fromDegrees(18),
            Rotation2d.fromDegrees(38)
        );

        public static final ArmPositionConstants kCubeLevel1Constants = new ArmPositionConstants(
            1.25,
            Rotation2d.fromDegrees(11),
            Rotation2d.fromDegrees(-16)
        );

        public static final ArmPositionConstants kCubeLevel2Constants = new ArmPositionConstants(
            11,
            Rotation2d.fromDegrees(23),
            Rotation2d.fromDegrees(-36)
        );
        
        public static final Rotation2d kTransformAmount = Rotation2d.fromDegrees(0.5);
        public static final double kArmRetractedPriorWaitDuration = 1; // seconds

        // public static final ArmPositionConstants kConeLevel3Constants = new ArmPositionConstants(
        //     -1,
        //     new Rotation2d(),
        //     new Rotation2d()
        // );

        // used to store target measurements for different arm states
        // arm constants assume pivot axle is origin and wrist axle is point
        // rotation2ds in this class are used assuming that:
        // - level with the ground is 0, and
        // - a positive value represents the arm moving up
        public static final class ArmPositionConstants extends ArmPosition {
            public ArmPositionConstants(double armRadius, Rotation2d armTheta, Rotation2d wristTheta) {
                super(armRadius, armTheta, wristTheta);
            }
        }
    }

    public static final class Intake {
        public static final int kIntakeMotorID = 18;

        public static final boolean kIsInverted = false;

        public static final double kInSpeed = 0.8;
        public static final double kOutSpeed = -0.2;

        public static final IntFunction<CANSparkMax> kMotorConstructor = (int ID) -> { return new CANSparkMax(ID, MotorType.kBrushless); };
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
        public static final int kDriverControllerID = 2;
        public static final int kOperatorControllerID = 4;

        public static final int kWindowButtonId = 7;
        public static final int kMenuButtonId = 8;

        public static final int kToggleFieldRelativeButton = 1;
        public static final int kResetYawButton = 8;
        public static final int kResetOdometryButton = 3;
        public static final int kSlowModeAxis = 3;

        public static final double kStickDeadband = 0.08;
        public static final double kTriggerDeadband = 0.08;
        public static final double kResponseCurveExponent = 5.0 / 3.0;

		public static final int kDefaultAutoIndex = 0;

    }
}