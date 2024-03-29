package com.spartronics4915.frc2023.commands;

import java.util.ArrayList;
import static com.spartronics4915.frc2023.Constants.Trajectory.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import static com.spartronics4915.frc2023.Constants.Swerve.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.spartronics4915.frc2023.subsystems.Swerve;

public class SwerveTrajectoryFollowerCommands {
	private final PIDController mXPID, mYPID;
	private final PIDController mThetaPID;
	private final Swerve mSwerve;

	public SwerveTrajectoryFollowerCommands() {
		mSwerve = Swerve.getInstance();
		mXPID = new PIDController(kLinearP, 0, 0);
		mYPID = new PIDController(kLinearP, 0, 0);
		mThetaPID = new PIDController(kLinearP, 0, 0);
		mThetaPID.enableContinuousInput(-Math.PI, Math.PI);
	}

	public class FollowDynamicTrajectory extends CommandBase {
		private PathPlannerTrajectory mTrajectory;
		private PPSwerveControllerCommand mControllerCommand;
		private Thread mTrajectoryThread;

		public FollowDynamicTrajectory(
			ArrayList<PathPoint> waypoints, // meters
			double maxVelocity, // meters per second
			double maxAccel // meters per second squared	
		) {
			mTrajectoryThread = new Thread(() -> {
				PathPlannerTrajectory traj = PathPlanner.generatePath(
					new PathConstraints(maxVelocity, maxAccel),
					waypoints
				);
				synchronized (this) {
					mTrajectory = traj;
				}
			});
			mTrajectoryThread.start();
			addRequirements(mSwerve);
		}
		
		public FollowDynamicTrajectory(
			ArrayList<PathPoint> waypoints // meters
		) {
			this(waypoints, kMaxVelocity, kMaxAccel);
		}

		@Override
		public void initialize() {
			if (mControllerCommand != null)
				mControllerCommand.initialize();
		}

		@Override
		public void execute() {
			if (mControllerCommand == null) 
				synchronized (this) {
					if (mTrajectory != null) {
						mControllerCommand = new PPSwerveControllerCommand(
							mTrajectory,
							mSwerve::getPose,
							kKinematics,
							mXPID, mYPID,
							mThetaPID,
							mSwerve::setModuleStates,
							true
						);
						mControllerCommand.initialize();
					}
				}
			
			if (mControllerCommand != null) {
				mControllerCommand.execute();
				
				// debug
				SmartDashboard.putNumber("Swerve xPID Setpoint", mXPID.getSetpoint());
				SmartDashboard.putNumber("Swerve xPID Position Error", mXPID.getPositionError());
				
				SmartDashboard.putNumber("Swerve yPID Setpoint", mYPID.getSetpoint());
				SmartDashboard.putNumber("Swerve yPID Position Error", mYPID.getPositionError());
				
				SmartDashboard.putNumber("Swerve thetaPID Setpoint", mThetaPID.getSetpoint());
				SmartDashboard.putNumber("Swerve thetaPID Position Error", mThetaPID.getPositionError());
			}
		}
		
		@Override
		public boolean isFinished() {
			return mControllerCommand != null && mControllerCommand.isFinished(); 
		}
		
		@Override
		public void end(boolean interrupted) {
			if (mControllerCommand != null)
				mControllerCommand.end(interrupted);
		}
	}
	
	public class FollowStaticTrajectory extends PPSwerveControllerCommand {
		public FollowStaticTrajectory(
			ArrayList<PathPoint> waypoints, // meters
			double maxVelocity, // meters per second
			double maxAccel // meters per second squared
		) {
			super(
				PathPlanner.generatePath(
					new PathConstraints(maxVelocity, maxAccel),
					waypoints
				),
				mSwerve::getPose,
				kKinematics,
				mXPID, mYPID,
				mThetaPID,
				mSwerve::setModuleStates,
				true,
				mSwerve
			);
		}

		public FollowStaticTrajectory(
			ArrayList<PathPoint> waypoints // meters
		) {
			this(waypoints, kMaxVelocity, kMaxAccel);
		}

		@Override
		public void execute() {
			super.execute();
			
			// debug
			SmartDashboard.putNumber("Swerve xPID Setpoint", mXPID.getSetpoint());
			SmartDashboard.putNumber("Swerve xPID Position Error", mXPID.getPositionError());
			
			SmartDashboard.putNumber("Swerve yPID Setpoint", mYPID.getSetpoint());
			SmartDashboard.putNumber("Swerve yPID Position Error", mYPID.getPositionError());
			
			SmartDashboard.putNumber("Swerve thetaPID Setpoint", mThetaPID.getSetpoint());
			SmartDashboard.putNumber("Swerve thetaPID Position Error", mThetaPID.getPositionError());
		}
	}
}