package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.Swerve;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static com.spartronics4915.frc2023.Constants.Swerve.*;
import static com.spartronics4915.frc2023.Constants.OI.*;

public class SwerveCommands {
    private final XboxController mController;

    private final Swerve mSwerve;
    private boolean mIsSlowMode = false;

    public SwerveCommands(XboxController controller, Swerve swerve) {
        mController = controller;
        mSwerve = swerve;
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

    public class ResetOdometry extends InstantCommand {
        public ResetOdometry() {

		}
		
		@Override
		public void initialize() {
			super.initialize();
            mSwerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
		}
    }

    public class ResetCommand extends InstantCommand {
        public ResetCommand() {
            addRequirements(mSwerve);
        }
		
		@Override
		public void initialize() {
			super.initialize();
			mSwerve.resetToAbsolute();
            mSwerve.resetYaw();
			mSwerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))); // for odometry testing
            mSwerve.stop();
			mSwerve.alignModules();
		}
    }

    public class TeleopCommand extends CommandBase {
        public TeleopCommand() {
            addRequirements(mSwerve);
        }

        @Override
        public void initialize() {}

        @Override
        public void execute() {
            double x1 = -mController.getLeftX();
            double y1 = mController.getLeftY();
            double x2 = -mController.getRightX();

            x1 = applyTransformations(x1);
            y1 = applyTransformations(y1);
            x2 = applyTransformations(x2);

            Translation2d translation = new Translation2d(-y1, -x1).times(kMaxSpeed);
            double rotation = -x2 * kMaxAngularSpeed;

            if (Math.abs(mController.getRawAxis(kSlowModeAxis)) <= kTriggerDeadband) { // <= for slow mode default
                translation = translation.times(kSlowModeSpeedMultiplier);
                rotation *= kSlowModeAngularSpeedMultiplier;
            }
            
            mSwerve.drive(translation, rotation, true);
        }

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    public class TestInitCommand extends CommandBase {
        public TestInitCommand() {
            addRequirements(mSwerve);
        }

        @Override
        public void initialize() {}

        @Override
        public void execute() {}

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public class TestCommand extends CommandBase {
        public TestCommand() {
            addRequirements(mSwerve);
        }

        @Override
        public void initialize() {}

        @Override
        public void execute() {}

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    private double applyTransformations(double c) {
        return applyResponseCurve(MathUtil.applyDeadband(c, kStickDeadband));
    }

    private double applyResponseCurve(double c) {
        return Math.signum(c) * Math.pow(Math.abs(c), kResponseCurveExponent);
    }

    public class RotateDegrees extends CommandBase {

        private double mDegreeRotate;
        public RotateDegrees(double degrees) {
            mDegreeRotate = degrees;
        }

        @Override
        public void initialize() {
            var yaw = mSwerve.getYaw();

            Rotation2d newYaw = yaw.plus(Rotation2d.fromDegrees(mDegreeRotate));
            var newCommand = new RotateToYaw(newYaw);
            newCommand.schedule();
        }
    }
    public class RotateToYaw extends CommandBase {

        private final double kMaxSpeedDegreesSec = 15;
        private final double mYawToleranceDegrees = 2;
        private final double ticDuration = 1.0 / 20;
        private Rotation2d mDestinationYaw;
        private Swerve mSwerveSubsystem;

        public RotateToYaw(Rotation2d destinationYaw) {

            
            mDestinationYaw = destinationYaw;
        }

        private double getYawToGo() {
            var currYaw = mSwerve.getYaw();

            return currYaw.minus(mDestinationYaw).getDegrees();
        }

        @Override
        public void execute() {

            var yawToGo = getYawToGo();
            double timeToFinish = Math.abs(yawToGo) / ticDuration;
            double currRotationSpeed = yawToGo / timeToFinish;

            if(currRotationSpeed > kMaxSpeedDegreesSec) {
                currRotationSpeed = kMaxSpeedDegreesSec;
            }

            Translation2d zeroTranslation = new Translation2d();
            double currRotationSpeedRadians = currRotationSpeed / 180 * Math.PI;

            mSwerve.drive(zeroTranslation, currRotationSpeedRadians, true);
        }

        @Override
        public boolean isFinished() {

            return (Math.abs(getYawToGo()) < mYawToleranceDegrees);
        }

    }

}
