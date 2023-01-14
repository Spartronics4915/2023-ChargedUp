package com.spartronics4915.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



import static com.spartronics4915.frc2023.Constants.Swerve.*;
import static com.spartronics4915.frc2023.Constants.Swerve.Module0.*;

public class TestSubsystem {
    private static final Integer driveMotorID = null;
    private static final String angleMotorID = null;
    private static final int encoderID = 0;
    private static final double angleOffset = 0;
    private final int mModuleNumber = 0;
    private double mAngleOffset;
    private double mLastAngle;

    private CANSparkMax mDriveMotor;
    private CANSparkMax mAngleMotor;

    private RelativeEncoder mDriveEncoder;
    private RelativeEncoder mIntegratedAngleEncoder;
    private AnalogEncoder mSteeringEncoder;

    private final SparkMaxPIDController mDriveController;
    private final SparkMaxPIDController mAngleController;

    private SwerveModuleState mDesiredState;

    private SimpleMotorFeedforward mFeedforward = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);

    public TestSubsystem() {
        
        mDriveMotor = kMotorConstructor.apply(kDriveMotorID);
        mDriveEncoder = mDriveMotor.getEncoder();
        mDriveController = mDriveMotor.getPIDController();
        configureDriveMotor();
        
        mAngleMotor = kMotorConstructor.apply(kAngleMotorID);
        mIntegratedAngleEncoder = mAngleMotor.getEncoder();
        mAngleController = mAngleMotor.getPIDController();
        configureAngleMotor();

        mSteeringEncoder = new AnalogEncoder(new AnalogInput(kEncoderID));
        mSteeringEncoder.setPositionOffset(angleOffset);
        confugureSteeringEncoder();

        mDesiredState = new SwerveModuleState();

        mLastAngle = getState().angle.getRadians();
    }


    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mDesiredState = desiredState;

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kMaxSpeed;
            mDriveMotor.set(percentOutput);
        } else {
            mDriveController.setReference(
                desiredState.speedMetersPerSecond,
                ControlType.kVelocity,
                0,
                mFeedforward.calculate(desiredState.speedMetersPerSecond)
            );
        }

        double angle = Math.abs(desiredState.speedMetersPerSecond) < kMaxSpeed * 0.01 ?
            mLastAngle :
            desiredState.angle.getRadians();

        // double angle = desiredState.angle.getRadians();

        mAngleController.setReference(angle, ControlType.kPosition);
        mLastAngle = angle;
    }

    public int getModuleNumber() {
        return mModuleNumber;
    }

    public void putSmartDashboardValues() {
        SmartDashboard.putNumber("mod " + mModuleNumber + " encoder", mSteeringEncoder.getDistance());
        SmartDashboard.putNumber("mod " + mModuleNumber + " encoder absolute", mSteeringEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("mod " + mModuleNumber + " integrated", mIntegratedAngleEncoder.getPosition());
        SmartDashboard.putNumber("mod " + mModuleNumber + " velocity", mDriveEncoder.getVelocity());

        SmartDashboard.putNumber("mod " + mModuleNumber + " desired angle", mDesiredState.angle.getRadians());
    }

    public void resetToAbsolute() {
        mIntegratedAngleEncoder.setPosition(mSteeringEncoder.getAbsolutePosition());
    }

    private void confugureSteeringEncoder() {
        mSteeringEncoder.setDistancePerRotation(2 * Math.PI);
        mSteeringEncoder.setPositionOffset(mAngleOffset);
    }

    private void configureDriveMotor() {
        mDriveMotor.restoreFactoryDefaults(); // ?
        mDriveMotor.setSmartCurrentLimit(Drive.kContinuousCurrentLimit);
        mDriveMotor.setInverted(kDriveMotorsAreInverted);
        mDriveMotor.setIdleMode(kDriveIdleMode);
        mDriveEncoder.setVelocityConversionFactor(Drive.kVelocityConversionFactor);
        mDriveEncoder.setPositionConversionFactor(Drive.kPositionConversionFactor);
        mDriveController.setP(Drive.kP);
        mDriveController.setI(Drive.kI);
        mDriveController.setD(Drive.kD);
        mDriveController.setFF(Drive.kFF);
        mDriveMotor.enableVoltageCompensation(kVoltageCompensation);
        mDriveMotor.burnFlash();
        mDriveEncoder.setPosition(0.0);
    }
    
    private void configureAngleMotor() {
        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.setSmartCurrentLimit(Angle.kContinuousCurrentLimit);
        mAngleMotor.setIdleMode(kAngleIdleMode);
        mIntegratedAngleEncoder.setPositionConversionFactor(Angle.kPositionConversionFactor);
        mAngleController.setP(Angle.kP);
        mAngleController.setI(Angle.kI);
        mAngleController.setD(Angle.kD);
        mAngleController.setFF(Angle.kFF);
        mAngleMotor.enableVoltageCompensation(kVoltageCompensation);
        mAngleMotor.burnFlash();
    }

    public SwerveModuleState getState() {
        double velocity = mDriveEncoder.getVelocity();
        Rotation2d angle = Rotation2d.fromDegrees(mIntegratedAngleEncoder.getPosition()); // TODO: why isnt this using the analog encoder
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double drivePosition = mDriveEncoder.getPosition();
        Rotation2d angle = getState().angle;
        return new SwerveModulePosition(drivePosition, angle);
    }
}