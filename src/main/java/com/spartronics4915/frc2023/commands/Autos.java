// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023.commands;

import java.util.function.Function;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.spartronics4915.frc2023.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static com.spartronics4915.frc2023.Constants.Trajectory.*;
import static com.spartronics4915.frc2023.Constants.Swerve.kKinematics;

public final class Autos {
    /** Example static factory for an autonomous command. */
    // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    //     return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
    // }

	public static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
		Swerve.getInstance()::getPose,
		Swerve.getInstance()::resetPose,
		kKinematics,
		new PIDConstants(0.1, 0.01, 0.01),
		new PIDConstants(0.1, 0.01, 0.01),
		Swerve.getInstance()::setModuleStates,
		kEventMap,
		true,
		Swerve.getInstance()
	);

	public static final PathPlannerTrajectory test2PieceTrajectory = PathPlanner.loadPath("Test 2-Piece", kPathConstraints);

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