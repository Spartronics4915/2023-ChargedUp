package com.spartronics4915.frc2023.commands;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

import com.fasterxml.jackson.databind.type.ArrayType;
import com.spartronics4915.frc2023.subsystems.ExtenderSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class ExtenderCommands {
    
    HashSet<Subsystem> mReq;
private final ExtenderSubsystem mExtender;
    public ExtenderCommands(ExtenderSubsystem mExtender) {
        this.mExtender = mExtender;
    
        mReq = new HashSet<Subsystem>();
        mReq.add(mExtender);
    }
    
    public class Extend extends CommandBase{
        public      Extend() {
            super();
        }

        @Override
        public Set<Subsystem> getRequirements() {
            
           return mReq; 
        }


        @Override
        public void initialize() {
            // TODO Auto-generated method stub
            super.initialize();
            mExtender.startExtending();
        }

        @Override
        public void end(boolean interrupted) {
            // TODO Auto-generated method stub
            super.end(interrupted);
            mExtender.stopMotor();
        }
    }

    public class Retract extends CommandBase{
        public Retract() {
            super();
        }

        @Override
        public void initialize() {
            // TODO Auto-generated method stub
            super.initialize();
            mExtender.startRetracting();
        }

        @Override
        public void end(boolean interrupted) {
            // TODO Auto-generated method stub
            super.end(interrupted);
            mExtender.stopMotor();
        }

        @Override
        public Set<Subsystem> getRequirements() {
            
           return mReq; 
        }

    }

   
}
