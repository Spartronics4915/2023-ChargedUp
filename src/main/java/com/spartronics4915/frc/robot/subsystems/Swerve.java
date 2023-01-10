package com.spartronics4915.frc2023.subsystems;

import static com.spartronics4915.frc2023.Constants.Swerve.kKinematics;
import static com.spartronics4915.frc2023.Constants.Swerve.kMaxSpeed;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;
import com.spartronics4915.frc2023.Constants.Swerve.Module0;
import com.spartronics4915.frc2023.Constants.Swerve.Module1;
import com.spartronics4915.frc2023.Constants.Swerve.Module2;
import com.spartronics4915.frc2023.Constants.Swerve.Module3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private SwerveDriveOdometry mOdometry;
    private SwerveModule[] mModules;
    private AHRS mNavX;

    private boolean mIsFieldRelative = true;

    public Swerve() {
        mNavX = new AHRS();
        mNavX.reset();

        mOdometry = new SwerveDriveOdometry(kKinematics, getYaw(), null);

        mModules = new SwerveModule[] {
            new SwerveModule(0, Module0.kConstants),
            new SwerveModule(1, Module1.kConstants),
            new SwerveModule(2, Module2.kConstants),
            new SwerveModule(3, Module3.kConstants)
        };
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        ChassisSpeeds chassisSpeeds;
        SmartDashboard.putBoolean("field relative", mIsFieldRelative);
        if (mIsFieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getYaw()
            );
        } else {
            chassisSpeeds = new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation
            );
        }

        SwerveModuleState[] moduleStates = kKinematics.toSwerveModuleStates(chassisSpeeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxSpeed);

        for (SwerveModule mod : mModules) {
            mod.setDesiredState(moduleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeed);

        for (SwerveModule mod : mModules) {
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
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

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(-mNavX.getYaw());
    }

    public void resetOdometry(Pose2d pose) {
        mOdometry.resetPosition(getYaw(), getPositions(), pose);
    }

    public void resetModuleZeroes() {
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

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mModules) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public void zeroNavX() {
        mNavX.reset();
    }

    public void zeroModules() {
        SwerveModuleState[] zeroedStates = new SwerveModuleState[4];
        Arrays.fill(zeroedStates, new SwerveModuleState(0, new Rotation2d(0)));
        // for (SwerveModuleState state : zeroedStates) {
        //     state = new SwerveModuleState(0, new Rotation2d(0));
        // }
        setModuleStates(zeroedStates);
    }

    @Override
    public void periodic() {
        mOdometry.update(getYaw(), getPositions());
        for (SwerveModule mod : mModules) {
            mod.putSmartDashboardValues();
        }
        SmartDashboard.putNumber("pose x", getPose().getX());
        SmartDashboard.putNumber("pose y", getPose().getY());
        SmartDashboard.putNumber("pose rotation degrees", getPose().getRotation().getDegrees());
    }
}