package com.spartronics4915.frc2023.commands;

import static com.spartronics4915.frc2023.Constants.Trajectory.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import static com.spartronics4915.frc2023.Constants.Swerve.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.spartronics4915.frc2023.subsystems.Swerve;

public class SwerveTrajectoryFollowerCommands {
	private SwerveTrajectoryFollowerCommands() {}
	
	private static final PIDController mXPID = new PIDController(kLinearP * 10, 0, 0);
	private static final PIDController mYPID = new PIDController(kLinearP * 10, 0, 0);
	private static final PIDController mThetaPID = new PIDController(0.1, 0.01, 0);
	
	public static class FollowTrajectoryCommand extends PPSwerveControllerCommand {
		public FollowTrajectoryCommand(PathPlannerTrajectory trajectory) {
			super(
				trajectory,
				Swerve.getInstance()::getPose,
				kKinematics,
				mXPID,
				mYPID,
				mThetaPID,
				Swerve.getInstance()::setModuleStates,
				true,
				Swerve.getInstance()
			);
		}
	}

	public static class FollowSingleTrajectoryCommand extends SequentialCommandGroup {
		public FollowSingleTrajectoryCommand(PathPlannerTrajectory trajectory) {
			super(
				new FollowTrajectoryCommand(trajectory),
				Swerve.getInstance().driveCommand(new ChassisSpeeds(), true, true)
			);
		}
	}
}