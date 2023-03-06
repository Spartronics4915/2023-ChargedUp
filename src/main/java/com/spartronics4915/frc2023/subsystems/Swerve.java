package com.spartronics4915.frc2023.subsystems;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix.sensors.BasePigeon;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2023.Constants.Swerve.*;
import static com.spartronics4915.frc2023.Constants.Camera.*;

import java.util.Arrays;

public class Swerve extends SubsystemBase {
    // private SwerveDriveOdometry mOdometry;
    private final SwerveDrivePoseEstimator mPoseEstimator;

    private SwerveModule[] mModules;

    private WPI_Pigeon2 mIMU;
    private Rotation2d mLastPitch;
    private Rotation2d mLastLastPitch;

	private final int mModuleCount;
    private PhotonCamera mFrontCamera;

    private boolean mIsFieldRelative = true;

    private static final boolean useCamera = false;

    private static Swerve mInstance = null;

    /**
     * Gets the current instance of the Swerve subsystem.
     * @return The current instance.
     */
    public static Swerve getInstance() {
        if (mInstance == null) {
            mInstance = new Swerve();
        }
        return mInstance;
    }

    private Swerve() {
        mIMU = new WPI_Pigeon2(kPigeonID);
        configurePigeon(mIMU);

        if (useCamera) {
            mFrontCamera = new PhotonCamera(NetworkTableInstance.getDefault(), kFrontCameraName);
        }

        mModules = new SwerveModule[] {
            new SwerveModule(0, Module0.kConstants),
            new SwerveModule(1, Module1.kConstants),
            new SwerveModule(2, Module2.kConstants),
            new SwerveModule(3, Module3.kConstants)
        };
		
		resetToAbsolute();

		mModuleCount = mModules.length;

		mPoseEstimator = new SwerveDrivePoseEstimator(
            kKinematics,
            getYaw(),
            getPositions(),
            new Pose2d(),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1)
        );
    }

    private void configurePigeon(WPI_Pigeon2 pigeon) {
		pigeon.configEnableCompass(false);
        
        pigeon.configMountPose(
            kPigeonMountPoseYaw,
            kPigeonMountPosePitch,
            kPigeonMountPoseRoll
		);
    }

    /**
     * Gets the module count.
     * @return The module count.
     */
	public int getModuleCount() {
		return mModuleCount;
	}

    /**
     * This overload should only be used for controller input.
     * @param translation The left stick x and y.
     * @param rotation The right stick x.
     * @param isOpenLoop Whether the drive motor doesn't use PID control
     */
    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        SmartDashboard.putBoolean("field relative", mIsFieldRelative);

        drive(chassisSpeeds, isOpenLoop);
    }

    /**
     * Drive the robot using a {@link ChassisSpeeds} based on the drivetrain's current field-oriented setting
     * @param chassisSpeeds The {@link ChassisSpeeds} to use.
     * @param isOpenLoop Whether the drive motor doesn't use PID control
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        drive(chassisSpeeds, mIsFieldRelative, isOpenLoop);
    }

    /**
     * Drive the robot using a {@link ChassisSpeeds}
     * @param chassisSpeeds The {@link ChassisSpeeds} to use.
     * @param fieldRelative Whether to drive in field-relative mode or not.
     * @param isOpenLoop Whether the drive motor doesn't use PID control
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative, boolean isOpenLoop) {
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getYaw());
        }

        chassisSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond * 0.7);

        SwerveModuleState[] moduleStates = kKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxSpeed);
        setModuleStates(moduleStates, isOpenLoop);
    }

    public CommandBase driveCommand(ChassisSpeeds chassisSpeeds, boolean fieldRelative, boolean isOpenLoop) {
        return runOnce(() -> drive(chassisSpeeds, fieldRelative, isOpenLoop));
    }

    /**
     * Set the module states. 
     * @param desiredStates The desired module states.
     * @param isOpenLoop Whether the drive motor doesn't use PID control
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeed);

        for (SwerveModule mod : mModules) {
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    /**
     * Set the module states.
     * @param desiredStates The desired module states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        setModuleStates(desiredStates, true);
    }

    public void setModuleStates(SwerveModuleState desiredState) {
        for (SwerveModule mod : mModules) {
            mod.setDesiredState(
                desiredState, true);
        }
    }

    /**
     * Force the modules to a specific orientation.
     * @param orientation The orientation to force the modules to. 
     * @param isOpenLoop Whether the drive motor doesn't use PID control
     */
    public void forceModuleOrientations(Rotation2d orientation, boolean isOpenLoop) {
        // Forces all of the modules to one orientation
        // Mainly for testing - be careful if you use it
        System.out.println("ForceOrientation!");
        System.out.println(orientation.toString());

        for (SwerveModule mod : mModules) {
            mod.forceModuleOrientation(orientation, isOpenLoop);
        }
    }
    
    public void setFieldRelative(boolean fieldRelative) {
        mIsFieldRelative = fieldRelative;
    }

    public void toggleFieldRelative() {
        mIsFieldRelative = !mIsFieldRelative;
    }

    public boolean getFieldRelative() {
        return mIsFieldRelative;
    }

    public WPI_Pigeon2 getIMU() {
        return mIMU;
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(mIMU.getYaw());
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(mIMU.getPitch());
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(mIMU.getRoll());
    }

    /**
     * Gets the rate of change of the pitch.
     * @return The rate of change of the pitch in rad/s.
     */
    public double getPitchOmega() {
        return (mLastPitch.minus(mLastLastPitch)).getRadians() / 0.02;
    }

    /**
     * Gets the robot's current chassis speeds.
     * @return The robot's speeds.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return kKinematics.toChassisSpeeds(getStates());
    }

    /**
     * Resets the pose estimator to the specified pose. 
     * @param pose The pose to reset the pose estimator to.
     */
    public void setPose(Pose2d pose) {
        mPoseEstimator.resetPosition(getYaw(), getPositions(), pose);
    }

    public Pose2d getPose() {
        return mPoseEstimator.getEstimatedPosition();
    }

	public Rotation2d getEstimatedYaw() {
		return getPose().getRotation();
	}

	public void setYaw(Rotation2d yaw) {
		mIMU.setYaw(yaw.getDegrees());
	}

	/**
	 * Resets odometry pose to yaw = 0
	 */
	public void resetYaw() {
		setYaw(new Rotation2d());
	}

    /**
     * Resets the modules' internal steering encoders to equal the absolute encoders.
     */
    public void resetToAbsolute() {
        for (SwerveModule mod : mModules) {
            mod.resetToAbsolute();
        }
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mModules) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModuleState[] getDesiredStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mModules) {
            states[mod.getModuleNumber()] = mod.getDesiredState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mModules) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Aligns the modules to the orientation 0.
     */
	public void alignModules() {
		forceModuleOrientations(Rotation2d.fromRadians(0), true);
	}

    public void stop() {
        SwerveModuleState[] zeroedStates = new SwerveModuleState[4];
        Arrays.fill(zeroedStates, new SwerveModuleState(0, new Rotation2d(0)));
        // for (SwerveModuleState state : zeroedStates) {
        //     state = new SwerveModuleState(0, new Rotation2d(0));
        // }
        setModuleStates(zeroedStates);
    }

    public SwerveModule[] getSwerveModules() {
        return mModules;
    }

	private class VisionMeasurement {
		public Pose2d mPose;
		public double mTime;

		public VisionMeasurement(Pose2d pose, double time) {
			mPose = pose;
			mTime = time;
		}
	};

	private VisionMeasurement getVisionMeasurement() {
        if (!useCamera) {
            return null;
        }
        var frontLatestResult = mFrontCamera.getLatestResult();
        if (frontLatestResult.hasTargets()) {
            double imageCaptureTime = (Timer.getFPGATimestamp() * 1000) - frontLatestResult.getLatencyMillis();
            var bestTarget = frontLatestResult.getBestTarget();
            int bestTargetID = bestTarget.getFiducialId();
            var camToTargetTransform3d = bestTarget.getBestCameraToTarget();
            var camToTargetTransform2d = new Transform2d(
                camToTargetTransform3d.getTranslation().toTranslation2d(),
                camToTargetTransform3d.getRotation().toRotation2d()
            );
            Pose2d camPose = kTagPoses[bestTargetID].transformBy(camToTargetTransform2d.inverse());
            SmartDashboard.putNumber("x to tag", camPose.getX());
            SmartDashboard.putNumber("y to tag", camPose.getY());
			return new VisionMeasurement(camPose.transformBy(kFrontCameraToRobot), imageCaptureTime);
		}
        return null;
	}

    private void updatePoseEstimator() {
		VisionMeasurement vision = getVisionMeasurement();
		if (vision != null)
			mPoseEstimator.addVisionMeasurement(vision.mPose, vision.mTime);
        mPoseEstimator.update(getYaw(), getPositions());
		SmartDashboard.putString("swervePose", mPoseEstimator.getEstimatedPosition().toString());
    }

    @Override
    public void periodic() {
        updatePoseEstimator();
        for (SwerveModule mod : mModules) {
            mod.putSmartDashboardValues();
        }
        SmartDashboard.putNumber("pose x", getPose().getX());
        SmartDashboard.putNumber("pose y", getPose().getY());
        SmartDashboard.putNumber("pose rotation degrees", getPose().getRotation().getDegrees());

        SmartDashboard.putBoolean("field relative", mIsFieldRelative);

        mLastLastPitch = mLastPitch;
        mLastPitch = getPitch();
    }
}