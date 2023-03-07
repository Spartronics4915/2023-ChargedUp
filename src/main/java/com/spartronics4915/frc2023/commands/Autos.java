// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import javax.sound.midi.Sequence;

import com.fasterxml.jackson.databind.introspect.AccessorNamingStrategy.Provider;
import com.pathplanner.lib.PathPoint;
import com.spartronics4915.frc2023.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
    /** Example static factory for an autonomous command. */
    // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    //     return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
    // }

	private final Swerve mSwerve;
	private final boolean mIsOpenLoop = true;
	private final SwerveCommands mSwerveCommands;
			

    public Autos(SwerveCommands swerveCommands) {
		mSwerve = Swerve.getInstance();
		mSwerveCommands = swerveCommands;
    }

	public class MoveForwardCommand extends SequentialCommandGroup {
		public MoveForwardCommand() {
			addRequirements(mSwerve);
			addCommands(
				new InstantCommand(() -> mSwerve.drive(new Translation2d(0, 0.1), 0, mIsOpenLoop)),
				new WaitCommand(1),
				new InstantCommand(() -> mSwerve.drive(new Translation2d(), 0, mIsOpenLoop))
			);
		}
	}

	public class Strategy {
		private final Function<Pose2d, CommandBase> mGetCommand;
		private final String mName;

		public Strategy(String name, Function<Pose2d, CommandBase> getCommand) {
			mName = name;
			mGetCommand = getCommand;
		}

		public String getName() {
			return mName;
		}

		public CommandBase getCommand(Pose2d initialPose) {
			return new SequentialCommandGroup(
				mSwerveCommands.new ResetCommand(initialPose),
				mGetCommand.apply(initialPose)
			);
		}
	}
}