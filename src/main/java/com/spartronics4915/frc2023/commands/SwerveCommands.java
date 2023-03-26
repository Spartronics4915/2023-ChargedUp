package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.Swerve;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import static com.spartronics4915.frc2023.Constants.Swerve.*;

import com.spartronics4915.frc2023.commands.DebugTeleopCommands.PIDWidget;

import static com.spartronics4915.frc2023.Constants.OI.*;

public class SwerveCommands {
    private final CommandXboxController mDriverController;

    private final Swerve mSwerve;
    private boolean mIsSprintMode = false;

    public SwerveCommands(CommandXboxController controller) {
        mDriverController = controller;
        mSwerve = Swerve.getInstance();
    }

    public class SetFieldRelative extends InstantCommand {
		private boolean mFieldRelative;

        public SetFieldRelative(boolean fieldRelative) {
			mFieldRelative = fieldRelative;
        }

		@Override
		public void initialize() {
			super.initialize();
			
            mSwerve.setFieldRelative(mFieldRelative);
		}
    }

    public class ToggleFieldRelative extends InstantCommand {
        public ToggleFieldRelative() {

		}

		@Override
		public void initialize() {
			super.initialize();
			mSwerve.toggleFieldRelative();
		}
    }

    public class ResetYaw extends InstantCommand {
        public ResetYaw() {

		}
		@Override
		public void initialize() {
			super.initialize();
			mSwerve.resetYaw();
		}
    }

    /**
     * Mainly for debugging, probably shouldn't be used during a match
     */
    public class ResetPose extends InstantCommand {
        public ResetPose() {
			
		}
		
		@Override
		public void initialize() {
			super.initialize();
            mSwerve.resetPose(new Pose2d(0, 0, new Rotation2d(0)));
		}
    }

    /**
     * Mainly used to initialize robot
     */
    public class ResetCommand extends InstantCommand {
		private Pose2d mInitialPose;
        public ResetCommand(Pose2d initialPose) {
            addRequirements(mSwerve);
			mInitialPose = initialPose;
		}
		
		@Override
		public void initialize() {
			super.initialize();
			mSwerve.resetToAbsolute();
            mSwerve.stop();
			mSwerve.alignModules();
			mSwerve.resetPose(new Pose2d());
		}
    }

    public class EnableSprintMode extends InstantCommand {
        public EnableSprintMode() {
            super(
                () -> mIsSprintMode = true
            );
        }
    }

    public class DisableSprintMode extends InstantCommand {
        public DisableSprintMode() {
            super(
                () -> mIsSprintMode = false
            );
        }
    }

    /**
     * The default teleop driving command for the swerve.
     */
    public class TeleopCommand extends CommandBase {
        public TeleopCommand() {
            addRequirements(mSwerve);
        }

        @Override
        public void initialize() {}

        @Override
        public void execute() {
            double x1 = mDriverController.getLeftX();
            double y1 = mDriverController.getLeftY();
            double x2 = mDriverController.getRightX();

            x1 = applyTransformations(x1);
            y1 = applyTransformations(y1);
            x2 = applyTransformations(x2);

            Translation2d translation = new Translation2d(-y1, -x1).times(kMaxSpeed);
            double rotation = -x2 * kMaxAngularSpeed;

            if (!mIsSprintMode) {
                translation = translation.times(kSlowModeSpeedMultiplier);
                rotation *= kSlowModeAngularSpeedMultiplier;
            }
            
            mSwerve.drive(translation, rotation, false);

            // if (!mIsSprintMode) {
            //     x1 *= kSlowModeSpeedMultiplier;
            //     y1 *= kSlowModeSpeedMultiplier;
            //     x2 *= kSlowModeAngularSpeedMultiplier;
            // }

            // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(-y1 * kMaxSpeed, -x1 * kMaxSpeed, x2 * kMaxAngularSpeed);
            // mSwerve.drive(chassisSpeeds, true);
        }

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    /**
     * Balance on the charge station.
     */
    public class Balance extends CommandBase {
        private final PIDController mXVelocityPIDController;
        private final PIDController mRotationPIDController;
        
        public Balance() {
            addRequirements(mSwerve);
            mXVelocityPIDController = new PIDController(
                BalanceConstants.XVelocityPID.kP,
                BalanceConstants.XVelocityPID.kI,
                BalanceConstants.XVelocityPID.kD
            );
            mRotationPIDController = new PIDController(
                BalanceConstants.ThetaPID.kP,
                BalanceConstants.ThetaPID.kI,
                BalanceConstants.ThetaPID.kD
            );
        }

        @Override
        public void initialize() {
            mXVelocityPIDController.setSetpoint(0);
            mRotationPIDController.setSetpoint(0);
        }

        @Override
        public void execute() {
            // double 
            
            double vx = mXVelocityPIDController.calculate(mSwerve.getPitch().getRadians());
            double omega = mRotationPIDController.calculate(mSwerve.getYaw().getRadians());

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, 0, omega);
            
            mSwerve.drive(chassisSpeeds, true, true);
        }

        @Override
        public void end(boolean interrupted) {

        }

        @Override
        public boolean isFinished() {
            return mXVelocityPIDController.atSetpoint() && Math.abs(mSwerve.getPitchOmega()) <= 0.1;
        }
    }

    private double applyTransformations(double c) {
        return applyResponseCurve(MathUtil.applyDeadband(c, kStickDeadband));
    }

    private double applyResponseCurve(double c) {
        return Math.signum(c) * Math.pow(Math.abs(c), kResponseCurveExponent);
    }

    public class RotateDegrees extends CommandBase {

        private Rotation2d mDegreeRotate;
        public RotateDegrees(Rotation2d rot) {
            mDegreeRotate = rot;
        }

        @Override
        public void initialize() {
            var yaw = mSwerve.getYaw();

            Rotation2d newYaw = yaw.plus(mDegreeRotate);
            var newCommand = new RotateToYaw(newYaw);
            newCommand.schedule();
        }
    }
    
    public class RotateToYaw extends CommandBase {

        private final double mYawToleranceDegrees = 2;
        private final double mAngularVelToleranceDegreesSec = 1;
        private Rotation2d mDestinationYaw;
        private final PIDWidget mPIDWidget;
        private final TrapezoidProfile.Constraints motionConstraints;
        public double mlastVelocity;

        public RotateToYaw(Rotation2d destinationYaw) {
            this(destinationYaw, null);
        }

        public RotateToYaw(Rotation2d destinationYaw, PIDWidget pidWidget) {
            mPIDWidget = pidWidget;
            motionConstraints = new TrapezoidProfile.Constraints(Math.PI / 4, Math.PI / 4);
            addRequirements(mSwerve);
            mlastVelocity = 0;
            mDestinationYaw = destinationYaw;
        }

        @Override
        public void initialize() {
        }
        

        @Override
        public void execute() {
            double yaw = mSwerve.getYaw().getRadians();
            double goal = mDestinationYaw.getRadians();
            var currState = new TrapezoidProfile.State(yaw, mlastVelocity);
            var goalState = new TrapezoidProfile.State(goal, 0);
            TrapezoidProfile currMotionProfile = new TrapezoidProfile(motionConstraints, goalState, currState);
            double ticLength = 1./50; // Robot runs at 50Hz
            var state = currMotionProfile.calculate(ticLength);
            mlastVelocity = state.velocity;
            if (mPIDWidget != null) {
                mPIDWidget.update(yaw, goal, state.velocity);
            }
            mSwerve.drive(
                new ChassisSpeeds(0, 0, state.velocity),
                true,
                true
            );
        }

        @Override

        public void end(boolean isInterrupted) {
            mSwerve.stop();
        }
        @Override
        public boolean isFinished() {
            double yaw = mSwerve.getYaw().getDegrees();
            double goal = mDestinationYaw.getDegrees();
            boolean positionFine = (Math.abs(yaw - goal) < mYawToleranceDegrees);
            // // boolean velocityFine = (Math.abs(pid.getVelocityError()) < pid.getVelocityTolerance());
            boolean finished = positionFine;
            if (finished) {
              System.out.println("done");
            }
            return finished;
        }
    }

    public class RotateYaw extends RotateToYaw {
        public RotateYaw(Rotation2d deltaYaw) {
            super(deltaYaw.plus(mSwerve.getYaw()));
        }

        public RotateYaw(Rotation2d deltaYaw, PIDWidget widget) {
            super(deltaYaw.plus(mSwerve.getYaw()), widget);
        }

    }
    public class DriveStraightToPoint extends CommandBase {

        Pose2d mTarget;
        public DriveStraightToPoint(Pose2d target) {

            mTarget = target;

        }

        @Override
        public void execute() {
            Pose2d currPose = mSwerve.getPose();
            Translation2d translationErrVec = mTarget.getTranslation().minus(currPose.getTranslation());
            double distToGoal = translationErrVec.getNorm();

        }
    }
}
