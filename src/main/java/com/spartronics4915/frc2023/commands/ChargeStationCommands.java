package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.sensors.Pigeon2;

public final class ChargeStationCommands {
    
    public static class AutoChargeStationClimb extends CommandBase{

        private Swerve mSwerveSubsystem;
        private Pigeon2 mIMU;
        public enum ClimbState { CLIMB_TO_GRIP, GRIP_TO_PLATFORM, LEVEL_ROBOT, STOP, ERROR}
        public ClimbState mCurrState, mLastState;
        private Timer mCurrStateTimer;
        private String mLogString;

        public AutoChargeStationClimb(Swerve swerveSubsystem, Pigeon2 IMU) {

            mSwerveSubsystem = swerveSubsystem;
            mIMU = IMU;
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
        }

        @Override
        public boolean isFinished() 
        {

            return (mCurrState==ClimbState.STOP) || (mCurrState==ClimbState.ERROR);
        }

        @Override
        public void execute() {
            switch (mCurrState) {

                case CLIMB_TO_GRIP:
                final double climb_to_grip_speed_m_s = 0.25;
                final double climb_to_grip_target_roll_deg = 25;
                final double climb_to_grip_time_allowed = 10;

                if(mCurrStateTimer.hasElapsed(climb_to_grip_time_allowed)) {
                    mSwerveSubsystem.stop();
                    mCurrState = ClimbState.ERROR;
                    mLogString = "Climb_To_Grip Timeout";
                }
                mSwerveSubsystem.setModuleStates(new SwerveModuleState(climb_to_grip_speed_m_s, Rotation2d.fromDegrees(0)));
                if (mSwerveSubsystem.getRoll().getDegrees() > climb_to_grip_target_roll_deg) {
                    mLastState = mCurrState;
                    mCurrState = ClimbState.GRIP_TO_PLATFORM;
                    mCurrStateTimer = new Timer();
                }

                case GRIP_TO_PLATFORM:

                default:
                mSwerveSubsystem.stop();
                mLogString = "Got to Default";
                mCurrState = ClimbState.ERROR;

            }
        }

    }
}
