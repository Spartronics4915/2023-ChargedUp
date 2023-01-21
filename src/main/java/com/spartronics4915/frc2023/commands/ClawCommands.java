// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.Constants.ArmConstants.ClawConstants;
import com.spartronics4915.frc2023.subsystems.ClawSubsystem;
import com.spartronics4915.frc2023.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** An example command that uses an example subsystem. */
public class ClawCommands {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final ClawSubsystem mSubsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ClawCommands(ClawSubsystem subsystem) {
        mSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
    }
    /**
     * InnerClawCommands
     */
    public class ClawGrabCommand extends SequentialCommandGroup{
        // Called when the command is initially scheduled.
        public ClawGrabCommand() {
            super();
            addRequirements(mSubsystem);
            addCommands(
            new InstantCommand(mSubsystem::motorIn),
            new ParallelRaceGroup(
                new WaitCommand(ClawConstants.kGrabTimerLength),
                new FunctionalCommand(()->{}, ()->{}, (Boolean interrupted)->{}, mSubsystem::limitSwitch)
            ),
            new InstantCommand(mSubsystem::motorOff)
            );
        }
    }
    public class ClawReleaseCommand extends SequentialCommandGroup{
        // Called when the command is initially scheduled.
        public ClawReleaseCommand() {
            super();
            addRequirements(mSubsystem);
            addCommands(
            new InstantCommand(mSubsystem::motorOut),
            new WaitCommand(ClawConstants.kReleaseTimerLength),
            new InstantCommand(mSubsystem::motorOff)
            );
        }
    }
}
