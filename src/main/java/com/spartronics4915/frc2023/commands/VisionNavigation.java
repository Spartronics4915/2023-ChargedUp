// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023.commands;


import com.spartronics4915.frc2023.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class VisionNavigation {
    public class AlignToTag extends CommandBase {
        @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
        private final Swerve m_Swerve;
        
        public AlignToTag(Swerve swerve) {
            m_Swerve = swerve;
            // Use addRequirements() here to declare subsystem dependencies.
            addRequirements(m_Swerve);
        }
        
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {}
        
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {}
        
        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {}
        
        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return false;
        }
    }
}