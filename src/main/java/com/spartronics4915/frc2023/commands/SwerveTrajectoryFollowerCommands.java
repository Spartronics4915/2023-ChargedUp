package com.spartronics4915.frc2023.commands;

import java.util.ArrayList;
import static com.spartronics4915.frc2023.Constants.Trajectory.*;
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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import com.spartronics4915.frc2023.subsystems.Swerve;

public class SwerveTrajectoryFollowerCommands {
	private final Swerve mSwerve;
	private final PIDController mXPID, mYPID;
	private final ProfiledPIDController mThetaPID;

	public SwerveTrajectoryFollowerCommands(Swerve swerve) {
		mSwerve = swerve;
		mXPID = new PIDController(kLinearP, 0, 0);
		mYPID = new PIDController(kLinearP, 0, 0);
		mThetaPID = new ProfiledPIDController(
			kThetaP, 0, 0, new TrapezoidProfile.Constraints(0, 0)
		);
		mThetaPID.enableContinuousInput(0, 2 * Math.PI);
	}

	private ProfiledPIDController updateThetaPID(
		double maxAngularVelocity, double maxAngularAcceleration // radians per second
	) {
		mThetaPID.setConstraints(new TrapezoidProfile.Constraints(
			maxAngularVelocity, maxAngularAcceleration
		));
		return mThetaPID;
	}

	class FollowTrajectory extends SwerveControllerCommand {
		public FollowTrajectory(
			ArrayList<Pose2d> waypoints, // meters
			double startVelocity, double endVelocity, double maxVelocity, // meters per second
			double maxAccel, // meters per second squared
			double maxAngularVelocity, double maxAngularAcceleration // radians per second
		) {
			super(
				TrajectoryGenerator.generateTrajectory( // FIXME: will possibly take longer than 1 cycle
					waypoints,
					new TrajectoryConfig(maxVelocity, maxAccel)
						.setStartVelocity(startVelocity)
						.setEndVelocity(endVelocity)
				),
				mSwerve::getPose,
				kKinematics,
				mXPID, mYPID,
				updateThetaPID(
					maxAngularVelocity,
					maxAngularAcceleration
				),
				mSwerve::setModuleStates,
				mSwerve
			);

		}

		@Override
		public void execute() {
			super.execute();
			SmartDashboard.putNumber("Swerve xPID Setpoint", mXPID.getSetpoint());
			SmartDashboard.putNumber("Swerve xPID Position Error", mXPID.getPositionError());
			
			SmartDashboard.putNumber("Swerve yPID Setpoint", mYPID.getSetpoint());
			SmartDashboard.putNumber("Swerve yPID Position Error", mYPID.getPositionError());
			
			// SmartDashboard.putNumber("Swerve thetaPID Setpoint", mThetaPID.getSetpoint());
			SmartDashboard.putNumber("Swerve thetaPID Position Error", mThetaPID.getPositionError());
		}
	}
}