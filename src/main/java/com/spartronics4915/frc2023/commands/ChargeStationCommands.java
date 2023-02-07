package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.sensors.Pigeon2;

public final class ChargeStationCommands {

    public static class AutoChargeStationClimb extends CommandBase {

        private Swerve mSwerveSubsystem;
        private Pigeon2 mIMU;

        public enum ClimbState {
            CLIMB_TO_GRIP, GRIP_TO_PLATFORM, LEVEL_ROBOT, STOP, ERROR
        }

        public ClimbState mCurrState, mLastState;
        private Timer mCurrStateTimer;
        private String mLogString;

        public AutoChargeStationClimb(Swerve swerveSubsystem) {
            addRequirements(swerveSubsystem);
            mSwerveSubsystem = swerveSubsystem;
            mIMU = mSwerveSubsystem.getIMU();
            mLogString = "";
        }

        @Override
        public void initialize() {
            mCurrState = ClimbState.CLIMB_TO_GRIP;
            mCurrStateTimer = new Timer();
            mLogString = "";
            mLastState = null;
        }

        @Override
        public void end(boolean interrupted) {
            mSwerveSubsystem.stop();
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
            final double climb_to_grip_speed_m_s = 1.5;
            switch (mCurrState) {

                case CLIMB_TO_GRIP:
                    final double climb_to_grip_target_pitch_deg = 10;
                    final double climb_to_grip_time_allowed = 5;

                    if (mCurrStateTimer.hasElapsed(climb_to_grip_time_allowed)) {
                        mSwerveSubsystem.stop();
                        mLastState = mCurrState;
                        mCurrState = ClimbState.ERROR;
                        mLogString = "Climb_To_Grip Timeout";
                    }
                    mSwerveSubsystem.drive(new ChassisSpeeds(climb_to_grip_speed_m_s, 0, 0), true, true);
                    if (Math.abs(mSwerveSubsystem.getPitch().getDegrees()) > climb_to_grip_target_pitch_deg) {
                        mLastState = mCurrState;
                        mCurrState = ClimbState.GRIP_TO_PLATFORM;
                        mCurrStateTimer = new Timer();
                        System.out.println("CLIMB_TO_GRIP success!");
                    }
                    break;

                case GRIP_TO_PLATFORM:
                    final double grip_to_platform_speed_m_s = climb_to_grip_speed_m_s * 0.5;
                    final double grip_to_platform_target_roll_deg = 8;
                    final double grip_to_platform_time_allowed = 5;
                    if (mCurrStateTimer.hasElapsed(grip_to_platform_time_allowed)) {
                        mSwerveSubsystem.stop();
                        mLastState = mCurrState;
                        mCurrState = ClimbState.ERROR;
                        mLogString = "grip_to_platform Timeout";
                    }
                    mSwerveSubsystem.drive(new ChassisSpeeds(grip_to_platform_speed_m_s, 0, 0), true, true);
                    if (Math.abs(mSwerveSubsystem.getPitch().getDegrees()) < grip_to_platform_target_roll_deg) {
                        mLastState = mCurrState;
                        mCurrState = ClimbState.STOP;
                        mCurrStateTimer = null;
                    }
                    break;

                default:
                    mSwerveSubsystem.stop();
                    mLogString = "Got to Default";
                    mLastState = mCurrState;
                    mCurrStateTimer = null;
                    mCurrState = ClimbState.ERROR;

            }
        }

    }
}
