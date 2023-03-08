package com.spartronics4915.frc2023.commands;

import static com.spartronics4915.frc2023.Constants.Trajectory.*;
import static com.spartronics4915.frc2023.Constants.Swerve.*;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import com.spartronics4915.frc2023.subsystems.Swerve;

public class SwerveTrajectoryFollowerCommands {
	private SwerveTrajectoryFollowerCommands() {}
	
	private static final PIDController mXPID = new PIDController(kLinearP, 0, 0);
	private static final PIDController mYPID = new PIDController(kLinearP, 0, 0);
	private static final ProfiledPIDController mThetaPID = new ProfiledPIDController(
		kThetaP, 
		0,
		0,
		new Constraints(2.5, 2.5)
	);

	private static final HolonomicDriveController mDriveController = new HolonomicDriveController(mXPID, mYPID, mThetaPID);
	
	public static class FollowTrajectoryCommand extends SwerveControllerCommand {
		public FollowTrajectoryCommand(Trajectory trajectory) {
			super(
				trajectory,
				Swerve.getInstance()::getPose,
				kKinematics,
				mDriveController,
				Swerve.getInstance()::setModuleStates,
				Swerve.getInstance()
			);
		}
	}

	public static class FollowSingleTrajectoryCommand extends SequentialCommandGroup {
		public FollowSingleTrajectoryCommand(Trajectory trajectory) {
			super(
				new FollowTrajectoryCommand(trajectory),
				Swerve.getInstance().driveCommand(new ChassisSpeeds(), true, true)
			);
		}
	}
}