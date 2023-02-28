package com.spartronics4915.frc2023.commands;

import static com.spartronics4915.frc2023.Constants.OI.kResponseCurveExponent;
import static com.spartronics4915.frc2023.Constants.OI.kStickDeadband;
import static com.spartronics4915.frc2023.Constants.Swerve.kMaxAcceleration;
import static com.spartronics4915.frc2023.Constants.Swerve.kMaxAngularSpeed;
import static com.spartronics4915.frc2023.Constants.Swerve.kMaxSpeed;
import static com.spartronics4915.frc2023.Constants.Swerve.kSlowModeAngularSpeedMultiplier;
import static com.spartronics4915.frc2023.Constants.Swerve.kSlowModeSpeedMultiplier;

import org.photonvision.targeting.PhotonPipelineResult;

import com.spartronics4915.frc2023.Constants.Swerve.BalanceConstants;
import com.spartronics4915.frc2023.commands.DebugTeleopCommands.PIDWidget;
import com.spartronics4915.frc2023.subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
    public class ResetOdometry extends InstantCommand {
        public ResetOdometry() {

		}
		
		@Override
		public void initialize() {
			super.initialize();
            mSwerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
		}
    }

    /**
     * Mainly used to initialize robot
     */
    public class ResetCommand extends InstantCommand {
        public ResetCommand() {
            addRequirements(mSwerve);
        }
		
		@Override
		public void initialize() {
			super.initialize();
			mSwerve.resetToAbsolute();
            mSwerve.stop();
			mSwerve.alignModules();
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
            
            mSwerve.drive(translation, rotation, true);

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
            
            mSwerve.drive(chassisSpeeds, true, false);
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
    

    public class RotateToTarget extends CommandBase {;
        // private final double mYawToleranceDegrees = 10;
        // private final double mAngularVelToleranceDegreesSec = 1;
        // private final ProfiledPIDController pid;

        public RotateToTarget() {
            // pid = new ProfiledPIDController(0.02, 0, 0.01, new TrapezoidProfile.Constraints(
            //     0.1,
            //     kMaxAcceleration
            // ));
            // pid.setTolerance(mYawToleranceDegrees, mAngularVelToleranceDegreesSec);
            // pid.enableContinuousInput(0, 360);

            addRequirements(mSwerve);
        }

        @Override
        public void initialize() {
            PhotonPipelineResult result = mSwerve.mCameraWrapper.photonCamera.getLatestResult();
            if (result.hasTargets()) {
                // var yaw = mSwerve.getYaw();
                // System.out.println("Current Yaw: " + yaw.getDegrees());
                double targetYaw = result.getBestTarget().getYaw();
                // double targetYaw = result.getBestTarget().getBestCameraToTarget().inverse().getTranslation().toTranslation2d().getAngle().getDegrees();
                System.out.println("Tag Yaw: " + targetYaw);
                // Rotation2d newYaw = yaw.minus(Rotation2d.fromDegrees(targetYaw));
                // System.out.println("Goal Yaw: " + newYaw.getDegrees());
                var newCommand = new RotateYaw(Rotation2d.fromDegrees(targetYaw));
                newCommand.schedule();
            }
        }

        // @Override
        // public void execute() {
        //     PhotonPipelineResult result = mSwerve.mCameraWrapper.photonCamera.getLatestResult();
        //     if (result.hasTargets()) {
        //         double tagYaw = result.getBestTarget().getYaw();
        //         // double d = pid.calculate(tagYaw, 0);
        //         // mSwerve.drive(
        //         //     new Translation2d(),
        //         //     -d,
        //         //     true
        //         // );
        //         System.out.println("Tag Yaw: " + tagYaw);
        //         System.out.println("Tag Pitch: " + result.getBestTarget().getPitch());
        //     }
        // }

        // @Override
        // public boolean isFinished() {
        //     boolean positionFine = (Math.abs(pid.getPositionError()) < pid.getPositionTolerance());
        //     boolean velocityFine = (Math.abs(pid.getVelocityError()) < pid.getVelocityTolerance());
        //     Boolean finished = positionFine && velocityFine;
        //     finished = false;
        //     if (finished) {
        //         System.out.println("done");
        //     }
        //     return finished;
        // }
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
            motionConstraints = new TrapezoidProfile.Constraints(Math.PI / 1, Math.PI / 2);
            addRequirements(mSwerve);
            mlastVelocity = 0;
            mDestinationYaw = destinationYaw.times(-1);
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
            double ticLength = 1./20; //Robot runs at 20Hz
            var driveCommand = currMotionProfile.calculate(ticLength);
            mlastVelocity = driveCommand.velocity;
            if (mPIDWidget != null) {
                mPIDWidget.update(yaw, goal, driveCommand.velocity);
            }
            mSwerve.drive(
                new ChassisSpeeds(0, 0, driveCommand.velocity),
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

    // public class RotateYaw extends CommandBase {

    //     private final double mYawToleranceDegrees = 10;
    //     private final double mAngularVelToleranceDegreesSec = 1;
    //     private final double kP = 0.2;
    //     private Rotation2d mDeltaYaw;
    //     private final ProfiledPIDController pid;

    //     public RotateYaw(Rotation2d deltaYaw) {
    //         pid = new ProfiledPIDController(kP, 0, 0.01, new TrapezoidProfile.Constraints(
    //             10,
    //             kMaxAcceleration
    //         ));
    //         pid.setTolerance(mYawToleranceDegrees, mAngularVelToleranceDegreesSec);
    //         pid.enableContinuousInput(0, 360);

    //         addRequirements(mSwerve);
    //         mDeltaYaw = deltaYaw;
    //     }

    //     @Override
    //     public void execute() {
    //         double d = pid.calculate(mDeltaYaw.getDegrees(), 0);
    //         //System.out.println(d);
    //         mSwerve.drive(
    //             new Translation2d(),
    //             -d,
    //             true
    //         );
    //     }

    //     @Override

    //     public void end(boolean isInterrupted) {
    //         mSwerve.stop();
    //     }
    //     @Override
    //     public boolean isFinished() {
    //         boolean positionFine = (Math.abs(pid.getPositionError()) < pid.getPositionTolerance());
    //         boolean velocityFine = (Math.abs(pid.getVelocityError()) < pid.getVelocityTolerance());
    //         Boolean finished = positionFine && velocityFine;
    //         if (finished) {
    //             System.out.println("done");
    //         }
    //         return finished;
    //     }

    // }

    public class RotateYaw extends RotateToYaw {
        public RotateYaw(Rotation2d deltaYaw) {
            super(mSwerve.getYaw().minus(deltaYaw));
        }

        // public RotateYaw(Rotation2d deltaYaw, PIDWidget widget) {
        //     super(deltaYaw.plus(mSwerve.getYaw()), widget);
        // }

    }
}
