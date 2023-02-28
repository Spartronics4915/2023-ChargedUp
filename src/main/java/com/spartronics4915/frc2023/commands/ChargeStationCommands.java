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

        public enum ClimbState {
            CLIMB_TO_GRIP, GRIP_TO_PLATFORM, LEVEL_ROBOT_SETUP, LEVEL_ROBOT, STOP, ERROR
        }

        public ClimbState mCurrState, mLastState;
        private Timer mCurrStateTimer;
        private String mLogString;

        public AutoChargeStationClimb() {
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

        /**
         * Only for testing
         */
        public AutoChargeStationClimb(ClimbState initialState) {
            this();
            mCurrState = initialState;
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
            final double climb_to_grip_speed_m_s = -2;
            switch (mCurrState) {

                case CLIMB_TO_GRIP: {
                    final double climb_to_grip_target_pitch_deg = 10;
                    final double climb_to_grip_time_allowed = 5;

                    if (mCurrStateTimer.hasElapsed(climb_to_grip_time_allowed)) {
                        mSwerve.stop();
                        mLastState = mCurrState;
                        mCurrState = ClimbState.ERROR;
                        mLogString = "Climb_To_Grip Timeout";
                    }
                    mSwerve.drive(new ChassisSpeeds(climb_to_grip_speed_m_s, 0, 0), true, true);
                    if (Math.abs(mSwerve.getPitch().getDegrees()) > climb_to_grip_target_pitch_deg) {
                        mLastState = mCurrState;
                        mCurrState = ClimbState.GRIP_TO_PLATFORM;
                        mCurrStateTimer = new Timer();
                        System.out.println("CLIMB_TO_GRIP success!");
                    }
                    break;
                }

                case GRIP_TO_PLATFORM: {
                    final double grip_to_platform_speed_m_s = climb_to_grip_speed_m_s * 0.25;
                    final double grip_to_platform_target_roll_deg = 8;
                    final double grip_to_platform_time_allowed = 5;
                    if (mCurrStateTimer.hasElapsed(grip_to_platform_time_allowed)) {
                        mSwerve.stop();
                        mLastState = mCurrState;
                        mCurrState = ClimbState.ERROR;
                        mLogString = "grip_to_platform Timeout";
                    }
                    mSwerve.drive(new ChassisSpeeds(grip_to_platform_speed_m_s, 0, 0), true, true);
                    if (Math.abs(mSwerve.getPitch().getDegrees()) < grip_to_platform_target_roll_deg) {
                        mLastState = mCurrState;
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
                    double vx = mVXPID.calculate(mSwerve.getPitch().getRadians());
                    double omega = mThetaPID.calculate(mSwerve.getYaw().getRadians());

                    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, 0, omega);

                    mSwerve.drive(chassisSpeeds, true, true);

                    if (mVXPID.atSetpoint() && Math.abs(mSwerve.getPitchOmega()) <= 0.1) {
                        mCurrState = ClimbState.STOP;
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
