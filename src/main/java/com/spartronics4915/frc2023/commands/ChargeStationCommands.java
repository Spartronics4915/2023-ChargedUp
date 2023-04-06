package com.spartronics4915.frc2023.commands;

import static com.spartronics4915.frc2023.Constants.Swerve.*;

import com.spartronics4915.frc2023.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public final class ChargeStationCommands {

    public static class AutoChargeStationClimb extends CommandBase {

        private final Swerve mSwerve;

        private final PIDController mVXPID;
        private final ProfiledPIDController mThetaPID;

        private final boolean mBackwards;
        private final boolean mMobility;

        public enum ClimbState {
            CLIMB_TO_GRIP_M,
            GO_OVER_PLATFORM,
            LAND_ON_GROUND,
            DRIVE_ON_GROUND,
            CLIMB_TO_GRIP,
            GRIP_TO_PLATFORM,
            LEVEL_ROBOT_SETUP,
            LEVEL_ROBOT,
            STOP,
            ERROR
        }

        public ClimbState mCurrState, mLastState;
        private Timer mCurrStateTimer;
        private String mLogString;

        /**
         * Defaults to backwards and no mobility
         */
        public AutoChargeStationClimb() {
			this(true);
        }

        public AutoChargeStationClimb(boolean backwards) {
            mBackwards = backwards;
            mMobility = false;
            mSwerve = Swerve.getInstance();
            addRequirements(mSwerve);
            mLogString = "";
            mCurrState = ClimbState.CLIMB_TO_GRIP;
            mVXPID = new PIDController(
                BalanceConstants.XVelocityPID.kP,
                BalanceConstants.XVelocityPID.kI,
                BalanceConstants.XVelocityPID.kD);
            mThetaPID = new ProfiledPIDController(
                BalanceConstants.ThetaPID.kP,
                BalanceConstants.ThetaPID.kI,
                BalanceConstants.ThetaPID.kD,
                new Constraints(kMaxAngularSpeed, kMaxAngularAcceleration));
        }

        public AutoChargeStationClimb(boolean backwards, boolean mobility) {
            mBackwards = mobility ? !backwards : backwards;
            mMobility = mobility;
            mSwerve = Swerve.getInstance();
            addRequirements(mSwerve);
            mLogString = "";
            mCurrState = ClimbState.CLIMB_TO_GRIP_M;
            mVXPID = new PIDController(
                BalanceConstants.XVelocityPID.kP,
                BalanceConstants.XVelocityPID.kI,
                BalanceConstants.XVelocityPID.kD);
            mThetaPID = new ProfiledPIDController(
                BalanceConstants.ThetaPID.kP,
                BalanceConstants.ThetaPID.kI,
                BalanceConstants.ThetaPID.kD,
                new Constraints(kMaxAngularSpeed, kMaxAngularAcceleration));
        }

        /**
         * Only for testing
         */
        public AutoChargeStationClimb(ClimbState initialState) {
            this();
            mCurrState = initialState;
        }

        private static double invertIf(boolean condition, double x) {
            return x * (condition ? -1. : 1.);
        }

        private double invertIfBackwards(double x) {
            return invertIf(mBackwards, x);
        }

        @Override
        public void initialize() {
            mCurrStateTimer = new Timer();
            mLogString = "";
            mLastState = null;
        }

        @Override
        public void end(boolean interrupted) {
            mSwerve.stop();
            if (interrupted) {
                mLastState = mCurrState;
                mLogString = "Interrupted";
            }
        }

        @Override
        public boolean isFinished() {
            return (mCurrState == ClimbState.STOP) || (mCurrState == ClimbState.ERROR);
        }

        @Override
        public void execute() {
            final double climb_to_grip_speed_m_s = invertIfBackwards(2.4);
            switch (mCurrState) {
                case CLIMB_TO_GRIP_M: {
                    final double targetPitchDegrees = 9;
                    
                    if (mCurrStateTimer.hasElapsed(2)) {
                        mSwerve.stop();
                        mLastState = mCurrState;
                        mCurrState = ClimbState.ERROR;
                        mLogString = "CLIMB_TO_GRIP_M timeout";
                    }
                    mSwerve.drive(new ChassisSpeeds(-climb_to_grip_speed_m_s, 0, 0), false, false);
                    if (Math.abs(mSwerve.getPitch().getDegrees()) > targetPitchDegrees) {
                        mLastState = mCurrState;
                        mCurrState = ClimbState.GO_OVER_PLATFORM;
                        mCurrStateTimer.reset();
                        System.out.println("CLIMB_TO_GRIP_M success");
                    }
                }

                case GO_OVER_PLATFORM: {
                    final double overPlatformSpeed = -invertIfBackwards(climb_to_grip_speed_m_s);
                    final double targetPitchDegrees = invertIfBackwards(-10);
                    if (mCurrStateTimer.hasElapsed(4)) {
                        mSwerve.stop();
                        mLastState = mCurrState;
                        mCurrState = ClimbState.ERROR;
                        mLogString = "GO_OVER_PLATFORM timeout";
                    }
                    mSwerve.drive(new ChassisSpeeds(overPlatformSpeed, 0, 0), false, false);
                    double p = mSwerve.getPitch().getDegrees();
                    if (mBackwards ? (p > targetPitchDegrees) : (p < targetPitchDegrees)) {
                        mLastState = mCurrState;
                        mCurrState = ClimbState.LAND_ON_GROUND;
                        System.out.println("GO_OVER_PLATFORM success");
                        mCurrStateTimer.reset();
                    }
                }

                case LAND_ON_GROUND: {
                    final double landOnGroundSpeed = -invertIfBackwards(1);
                    final double landOnGroundTargetPitchDeg = 1;
                    if (mCurrStateTimer.hasElapsed(2)) {
                        mSwerve.stop();
                        mLastState = mCurrState;
                        mCurrState = ClimbState.ERROR;
                        mLogString = "LAND_ON_GROUND timeout";
                    }
                    mSwerve.drive(new ChassisSpeeds(landOnGroundSpeed, 0, 0));
                    if (Math.abs(mSwerve.getPitch().getDegrees()) < landOnGroundTargetPitchDeg) {
                        mLastState = mCurrState;
                        mCurrState = ClimbState.DRIVE_ON_GROUND;
                        System.out.println("LAND_ON_GROUND success");
                        mCurrStateTimer.reset();
                    }
                }

                case DRIVE_ON_GROUND: {
                    final double groundSpeed = -invertIfBackwards(1);
                    mSwerve.drive(new ChassisSpeeds(groundSpeed, 0, 0));
                    if (mCurrStateTimer.hasElapsed(1)) {
                        mLastState = mCurrState;
                        mCurrState = ClimbState.CLIMB_TO_GRIP;
                        System.out.println("DRIVE_ON_GROUND success");
                    }
                }

                case CLIMB_TO_GRIP: {
                    final double climb_to_grip_target_pitch_deg = 9;
                    final double climb_to_grip_time_allowed = 5;

                    if (mCurrStateTimer.hasElapsed(climb_to_grip_time_allowed)) {
                        mSwerve.stop();
                        mLastState = mCurrState;
                        mCurrState = ClimbState.ERROR;
                        mLogString = "Climb_To_Grip Timeout";
                    }
                    mSwerve.drive(new ChassisSpeeds(climb_to_grip_speed_m_s, 0, 0), false, false);
                    if (Math.abs(mSwerve.getPitch().getDegrees()) > climb_to_grip_target_pitch_deg) {
                        mLastState = mCurrState;
                        mCurrState = ClimbState.GRIP_TO_PLATFORM;
                        mCurrStateTimer = new Timer();
                        System.out.println("CLIMB_TO_GRIP success!");
                    }
                    break;
                }

                case GRIP_TO_PLATFORM: {
                    final double grip_to_platform_speed_m_s = invertIfBackwards(2. * 0.3); // 0.1875
                    final double grip_to_platform_target_roll_deg = 7;
                    final double grip_to_platform_time_allowed = 5;
                    if (mCurrStateTimer.hasElapsed(grip_to_platform_time_allowed)) {
                        mSwerve.stop();
                        mLastState = mCurrState;
                        mCurrState = ClimbState.ERROR;
                        mLogString = "grip_to_platform Timeout";
                    }
                    mSwerve.drive(new ChassisSpeeds(grip_to_platform_speed_m_s, 0, 0), false, false);
                    if (Math.abs(mSwerve.getPitch().getDegrees()) < grip_to_platform_target_roll_deg) {
                        mLastState = mCurrState;
                        System.out.println("Switching to LEVEL_ROBOT");
                        mCurrState = ClimbState.LEVEL_ROBOT_SETUP;
                        mCurrStateTimer = null;
                    }
                    break;
                }

                case LEVEL_ROBOT_SETUP: {
                    mVXPID.setSetpoint(0);
                    mThetaPID.setGoal(new State(0, 0));
                    mCurrState = ClimbState.LEVEL_ROBOT;
                }

                case LEVEL_ROBOT: {
                    // double vx = mVXPID.calculate(mSwerve.getPitch().getRadians());
                    // double omega = mThetaPID.calculate(mSwerve.getYaw().getRadians());

                    // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, 0, omega);

                    // mSwerve.drive(chassisSpeeds, true, true);

                    // if (mVXPID.atSetpoint() && Math.abs(mSwerve.getPitchOmega()) <= 0.1) {
                    //     mCurrState = ClimbState.STOP;
                    // }
                    // break;

                    final double pitchControlThreshold = 6;
                    final double vXAbsMetersPerSecond = 0.3;

                    if (Math.abs(mSwerve.getPitch().getDegrees()) > pitchControlThreshold) {
                        ChassisSpeeds c = new ChassisSpeeds(Math.copySign(vXAbsMetersPerSecond, mSwerve.getPitch().getDegrees()), 0, 0);
                        mSwerve.drive(c, false, false);
                    } else {
                        mSwerve.drive(new ChassisSpeeds());
                    }

                    break;
                }

                default: {
                    mSwerve.stop();
                    mLogString = "Got to Default";
                    mLastState = mCurrState;
                    mCurrStateTimer = null;
                    mCurrState = ClimbState.ERROR;
                }
            }
        }

    }
}
