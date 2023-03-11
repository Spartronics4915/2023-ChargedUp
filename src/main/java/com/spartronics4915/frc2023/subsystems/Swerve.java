package com.spartronics4915.frc2023.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;

import com.ctre.phoenix.sensors.BasePigeon;
import com.ctre.phoenix.sensors.Pigeon2;
import com.spartronics4915.frc2023.Constants.Swerve.Module0;
import com.spartronics4915.frc2023.Constants.Swerve.Module1;
import com.spartronics4915.frc2023.Constants.Swerve.Module2;
import com.spartronics4915.frc2023.Constants.Swerve.Module3;
import com.spartronics4915.frc2023.Constants.NodePoseConstants;
import com.spartronics4915.frc2023.PhotonCameraWrapper;
import com.spartronics4915.frc2023.PhotonCameraWrapper.VisionMeasurement;
import com.spartronics4915.frc2023.commands.PrintPos;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    
    public PhotonCameraWrapper mCameraWrapper;
    // private PhotonCamera mFrontCamera;

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

        mCameraWrapper = new PhotonCameraWrapper();
        // mFrontCamera = new PhotonCamera(NetworkTableInstance.getDefault(), kFrontCameraName);

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

        SwerveModuleState[] moduleStates = kKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxSpeed);
        setModuleStates(moduleStates, isOpenLoop);
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

    // public void zeroPIDP(){
    //     for (var module: getSwerveModules()) {

    //         module.zeroPIDP();
    //     }
    // }


    public void updatePoseEstimator() {
		VisionMeasurement vision = mCameraWrapper.getEstimatedGlobalPose();
        mPoseEstimator.update(getYaw(), getPositions());
		if (vision != null)
			mPoseEstimator.addVisionMeasurement(vision.mPose, vision.mTime);
		SmartDashboard.putString("swervePose", mPoseEstimator.getEstimatedPosition().toString());
    }

    /**
     * Checks if a given pose is within the alignment area in,
     * which is past the charge station within the community
     * @param pose The pose you wish to check
     * @return If the pose is within the alignment area
     */
    private static boolean getAbleToAlign(Pose2d pose) {
        //TODO: what values are within community?
        //TODO: should this be in this file?
    }

    /**
     * Aligns the robot to the nearest node, provided the robot is
     * within the alignment area
     * @return Whether the operation was successful
     */
    public boolean alignToNearestNode() {
        Pose2d currentPose = mPoseEstimator.getEstimatedPosition();
        if (!getAbleToAlign(currentPose)) return false; //if we are outside the area we cannot align
        // There's already a getCone() method in Swerve Commands which gets the nearest
        // cone node. I think it would be best to just iterate on this to include
        // cube nodes and use that
        // Alliance currentAlliance = DriverStation.getAlliance(); //our alliance determines the values
        // Pose2d[] nodePoses = (currentAlliance == Alliance.Red ?
        //     NodePoseConstants.redAlliance :
        //     NodePoseConstants.blueAlliance); //this doesnt account for if alliance is Invalid
        // Pose2d desiredPose = currentPose.nearest(Arrays.asList(nodePoses)); //gets the closest node
    }

    @Override
    public void periodic() {
        updatePoseEstimator();
        resetToAbsolute();
        
        for (SwerveModule mod : mModules) {
            mod.putSmartDashboardValues();
        }
        SmartDashboard.putNumber("pose x", getPose().getX());
        SmartDashboard.putNumber("pose y", getPose().getY());
        SmartDashboard.putNumber("pose rotation degrees", getPose().getRotation().getDegrees());
        
        SmartDashboard.putNumber("Yaw", getYaw().getDegrees());

        SmartDashboard.putBoolean("field relative", mIsFieldRelative);

        mLastLastPitch = mLastPitch;
        mLastPitch = getPitch();
    }
}