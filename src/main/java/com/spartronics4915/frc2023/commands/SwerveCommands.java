package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.PhotonCameraWrapper;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static com.spartronics4915.frc2023.Constants.Swerve.*;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import static com.spartronics4915.frc2023.Constants.OI.*;

public class SwerveCommands {
    private final CommandXboxController mDriverController;

    private final Swerve mSwerve;
    private boolean mIsSprintMode = false;

    public SwerveCommands(CommandXboxController controller, Swerve swerve) {
        mDriverController = controller;
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

            Translation2d translation = new Translation2d(-y1, x1).times(kMaxSpeed);
            double rotation = x2 * kMaxAngularSpeed;

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
    public class RotateToTarget extends CommandBase {
        private PhotonCameraWrapper cameraWrapper;
        private final double mYawToleranceDegrees = 10;
        private final double mAngularVelToleranceDegreesSec = 1;
        private final double kP = 0.2;
        private final ProfiledPIDController pid;

        public RotateToTarget(PhotonCameraWrapper camera) {
            cameraWrapper = camera;
            pid = new ProfiledPIDController(kP, 0, 0.01, new TrapezoidProfile.Constraints(
                10,
                kMaxAcceleration
            ));
            pid.setTolerance(mYawToleranceDegrees, mAngularVelToleranceDegreesSec);
            pid.enableContinuousInput(0, 360);

            addRequirements(mSwerve);
        }

        // @Override
        // public void initialize() {
        //     PhotonPipelineResult result = cameraWrapper.photonCamera.getLatestResult();
        //     if (result.hasTargets()) {
        //         // var yaw = mSwerve.getYaw();
        //         // System.out.println("Current Yaw: " + yaw.getDegrees());
        //         // double targetYaw = result.getBestTarget().getYaw();
        //         double targetYaw = result.getBestTarget().getBestCameraToTarget().inverse().getTranslation().toTranslation2d().getAngle().getDegrees();
        //         // System.out.println("Tag Yaw: " + targetYaw);
        //         // Rotation2d newYaw = yaw.minus(Rotation2d.fromDegrees(targetYaw));
        //         // System.out.println("Goal Yaw: " + newYaw.getDegrees());
        //         var newCommand = new RotateYaw(Rotation2d.fromDegrees(targetYaw));
        //         newCommand.schedule();
        //     }
        // }

        @Override
        public void execute() {
            PhotonPipelineResult result = cameraWrapper.photonCamera.getLatestResult();
            if (result.hasTargets()) {
                double tagYaw = result.getBestTarget().getYaw();
                double d = pid.calculate(tagYaw, 0);
                mSwerve.drive(
                    new ChassisSpeeds(
                        0,
                        0,
                        -d
                    ),
                    true
            );
            }
        }

        @Override
        public boolean isFinished() {
            boolean positionFine = (Math.abs(pid.getPositionError()) < pid.getPositionTolerance());
            boolean velocityFine = (Math.abs(pid.getVelocityError()) < pid.getVelocityTolerance());
            Boolean finished = positionFine && velocityFine;
            if (finished) {
                System.out.println("done");
            }
            return finished;
        }
    }

    public class RotateToYaw extends CommandBase {

        private final double kMaxSpeedDegreesSec = 15;
        private final double mYawToleranceDegrees = 10;
        private final double mAngularVelToleranceDegreesSec = 1;
        private final double ticDuration = 1.0 / 20;
        private final double kP = 0.2;
        private Rotation2d mDestinationYaw;
        private final ProfiledPIDController pid;

        public RotateToYaw(Rotation2d destinationYaw) {
            pid = new ProfiledPIDController(kP, 0, 0.01, new TrapezoidProfile.Constraints(
                10,
                kMaxAcceleration
            ));
            pid.setTolerance(mYawToleranceDegrees, mAngularVelToleranceDegreesSec);
            pid.enableContinuousInput(0, 360);

            addRequirements(mSwerve);
            mDestinationYaw = destinationYaw;
        }

        private double getYawToGo() {
            var currYaw = mSwerve.getYaw();

            return currYaw.minus(mDestinationYaw).getDegrees();
        }
//zack was here
        @Override
        public void execute() {
            double d = pid.calculate(mSwerve.getYaw().getDegrees(), mDestinationYaw.getDegrees());
            //System.out.println(d);
            mSwerve.drive(
                new ChassisSpeeds(
                    0,
                    0,
                    -d
                ),
                true
            );
            // mSwerve.drive(
            //     new Translation2d(),
            //     pid.calculate(
            //         mSwerve.getYaw().getDegrees(),
            //         mDestinationYaw.getDegrees()
            //     ), true
            // );
            // var yawToGo = getYawToGo();
            // double timeToFinish = Math.abs(yawToGo) / ticDuration;
            // double currRotationSpeed = yawToGo / timeToFinish;

            // if(currRotationSpeed > kMaxSpeedDegreesSec) {
            //     currRotationSpeed = kMaxSpeedDegreesSec;
            // }

            // Translation2d zeroTranslation = new Translation2d();
            // double currRotationSpeedRadians = currRotationSpeed / 180 * Math.PI;

            // mSwerve.drive(zeroTranslation, currRotationSpeedRadians, true);
        }

        @Override
        public boolean isFinished() {
            boolean positionFine = (Math.abs(pid.getPositionError()) < pid.getPositionTolerance());
            boolean velocityFine = (Math.abs(pid.getVelocityError()) < pid.getVelocityTolerance());
            // try {
            //     Field controlfield = ProfiledPIDController.class.getDeclaredField("m_controller");
            //     controlfield.setAccessible(true);
            //     PIDController control = (PIDController)controlfield.get(pid);
            //     Field hasMeasurementField = PIDController.class.getDeclaredField("m_haveMeasurement");
            //     hasMeasurementField.setAccessible(true);
            //     boolean hasMeasurement = (boolean)hasMeasurementField.get(control);
                
            //     Field hasSetpointField = PIDController.class.getDeclaredField("m_haveSetpoint");
            //     hasSetpointField.setAccessible(true);
            //     boolean hasSetpoint = (boolean)hasSetpointField.get(control);
            //     System.out.println(hasMeasurement + " " + hasSetpoint + " " + pid.atSetpoint());
            // } catch (Exception e) {
            //     // TODO Auto-generated catch block
            //     e.printStackTrace();
            // }
            Boolean finished = positionFine && velocityFine;
            if (finished) {
                System.out.println("done");
            }
            return finished;
        }

    }

    public class RotateYaw extends CommandBase {

        private final double mYawToleranceDegrees = 10;
        private final double mAngularVelToleranceDegreesSec = 1;
        private final double kP = 0.2;
        private Rotation2d mDeltaYaw;
        private final ProfiledPIDController pid;

        public RotateYaw(Rotation2d deltaYaw) {
            pid = new ProfiledPIDController(kP, 0, 0.01, new TrapezoidProfile.Constraints(
                10,
                kMaxAcceleration
            ));
            pid.setTolerance(mYawToleranceDegrees, mAngularVelToleranceDegreesSec);
            pid.enableContinuousInput(0, 360);

            addRequirements(mSwerve);
            mDeltaYaw = deltaYaw;
        }

        @Override
        public void execute() {
            double d = pid.calculate(mDeltaYaw.getDegrees(), 0);
            //System.out.println(d);
            mSwerve.drive(
                new ChassisSpeeds(
                    0,
                    0,
                    -d
                ),
                true
            );
        }

        @Override
        public boolean isFinished() {
            boolean positionFine = (Math.abs(pid.getPositionError()) < pid.getPositionTolerance());
            boolean velocityFine = (Math.abs(pid.getVelocityError()) < pid.getVelocityTolerance());
            Boolean finished = positionFine && velocityFine;
            if (finished) {
                System.out.println("done");
            }
            return finished;
        }

    }

}
