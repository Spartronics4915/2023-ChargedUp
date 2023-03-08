package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.ExtenderComponent;
import com.spartronics4915.frc2023.subsystems.ExtenderComponent;

import edu.wpi.first.wpilibj2.command.CommandBase;

public final class ExtenderCommands {
    
    private final ExtenderComponent mExtender;
    public ExtenderCommands(ExtenderComponent mExtender) {
        this.mExtender = mExtender;
    }
    
    public class Extend extends CommandBase{
        public Extend() {
            super();
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
    }

   
}
