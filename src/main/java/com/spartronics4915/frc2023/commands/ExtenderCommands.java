package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.ExtenderSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public final class ExtenderCommands {
    
    public static class ExtendNInches extends CommandBase{

        private double targetPos;
        private ExtenderSubsystem mSubsystem;
        private double mDistToExtend;
        public ExtendNInches(double N, ExtenderSubsystem subsystem) {
            mSubsystem = subsystem;
            mDistToExtend = N;
        }

        @Override
        public void initialize() {

            targetPos = mSubsystem.getPosition() + mDistToExtend;
            mSubsystem.targetReference = targetPos;
        }

        @Override
        public void execute() {
            double currPos = mSubsystem.getPosition();
            if (currPos < targetPos) {
                mSubsystem.startExtending();
            } else {
                mSubsystem.startRetracting();
            }
        }
        @Override
        public void end(boolean isInterrupted) {
            mSubsystem.stopMotor();
        }

        @Override
        public boolean isFinished() {
            return (Math.abs(mSubsystem.getPosition() - targetPos) < 0.05);
        }
    }
}
