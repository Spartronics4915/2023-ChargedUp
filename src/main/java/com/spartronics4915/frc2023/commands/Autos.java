// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023.commands;

import java.util.ArrayList;
import java.util.List;

import javax.sound.midi.Sequence;

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
	private final SwerveTrajectoryFollowerCommands mSwerveTrajectoryFollowerCommands;
	private final double maxVelocity = 0.5;
	private final double maxAccel = 1;
	private final double maxAngularVelocity = 0.8;
	private final double maxAngularAcceleration = 0.2;
			

    public Autos(SwerveTrajectoryFollowerCommands swerveTrajectoryFollowerCommands) {
		mSwerve = Swerve.getInstance();
		mSwerveTrajectoryFollowerCommands = swerveTrajectoryFollowerCommands;
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

	public class MoveForwardCommandFancy extends SequentialCommandGroup {
		public MoveForwardCommandFancy() {
			addRequirements(mSwerve);
			addCommands(
				mSwerveTrajectoryFollowerCommands.new FollowTrajectory(
					new ArrayList<>(List.of(
						new PathPoint(new Translation2d(0, 0), new Rotation2d(0)),
						new PathPoint(new Translation2d(1, 0), new Rotation2d(Math.PI / 2))
					)),
					maxVelocity, maxAccel
				),
				new InstantCommand(() -> {
					mSwerve.drive(new Translation2d(), 0, mIsOpenLoop);
					for (int i = 0; i < 100; i++) System.out.println("Finally finished the gauntlet!");
				})
			);
		}
	}

	public class MoveBackAndForthFancy extends SequentialCommandGroup {
		public MoveBackAndForthFancy() {
			PathPoint aprilTag1 = new PathPoint(new Translation2d(0, 0), new Rotation2d(Math.PI / 2));
			PathPoint aprilTag2 = new PathPoint(new Translation2d(0, 6), new Rotation2d(-Math.PI / 2));
			addCommands(
				mSwerveTrajectoryFollowerCommands.new FollowTrajectory(
					new ArrayList<>(List.of(
						aprilTag1,
						aprilTag2
					)),
					maxVelocity, maxAccel
				),
				mSwerveTrajectoryFollowerCommands.new FollowTrajectory(
					new ArrayList<>(List.of(
						aprilTag2,
						aprilTag1
					)),
					maxVelocity, maxAccel
				)
			);
		}
	}
}