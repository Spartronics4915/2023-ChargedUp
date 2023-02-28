package com.spartronics4915.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.spartronics4915.frc2023.Constants.Swerve.*;

import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
	public interface AbsoluteEncoder {
		// degrees
		public double getAbsolutePosition();
	}

	public static class AbsoluteCANCoder implements AbsoluteEncoder {
		private final CANCoder mCANCoder;
		public AbsoluteCANCoder(int id) {
			mCANCoder = new CANCoder(id);
		}

		public double getAbsolutePosition() {
			return mCANCoder.getAbsolutePosition();
		}
	}

	public static class AbsoluteAnalogEncoder implements AbsoluteEncoder {
		private final AnalogEncoder mAnalogEncoder;
		public AbsoluteAnalogEncoder(int id) {
			mAnalogEncoder = new AnalogEncoder(new AnalogInput(id));
		}

		public double getAbsolutePosition() {
			return 360 * (1.0 - mAnalogEncoder.getAbsolutePosition());
		}
	}

    private final int mModuleNumber;
    private final Rotation2d mAbsoluteOffset;
    private double mLastAngle;

    private final CANSparkMax mDriveMotor;
    private final CANSparkMax mAngleMotor;

    private final RelativeEncoder mDriveEncoder;
    private final RelativeEncoder mIntegratedAngleEncoder;
    private final CANCoder mAngleEncoder;

    private final SparkMaxPIDController mDriveController;
    private final SparkMaxPIDController mAngleController;

    private SwerveModuleState mDesiredState;

    private final SimpleMotorFeedforward mFeedforward = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);

    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int encoderID, double absoluteOffsetRadians) {
        mModuleNumber = moduleNumber;

        mAbsoluteOffset = Rotation2d.fromRadians(absoluteOffsetRadians);
        
        mDriveMotor = kMotorConstructor.apply(driveMotorID);
        mDriveEncoder = mDriveMotor.getEncoder();
        mDriveController = mDriveMotor.getPIDController();
        configureDriveMotor();
        
        mAngleMotor = kMotorConstructor.apply(angleMotorID);
        mIntegratedAngleEncoder = mAngleMotor.getEncoder();
        mAngleController = mAngleMotor.getPIDController();
        configureAngleMotor();

        mAngleEncoder = new CANCoder(encoderID);

        resetToAbsolute();

        mDesiredState = new SwerveModuleState();

        mLastAngle = getState().angle.getRadians();
    }

    public SwerveModule(int moduleNumber, SwerveModuleConstants constants) {
        this(moduleNumber, constants.driveMotorID, constants.angleMotorID, constants.encoderID, constants.absoluteOffset);
    }

    public void forceModuleOrientation(Rotation2d newAngle, boolean isOpenLoop){
        // Forces all of the modules to a desired orientation.  Will not change the speed
        // Mainly for testing, be careful if you use this.

        var currentState = this.getState();
        var newState = new SwerveModuleState(currentState.speedMetersPerSecond, newAngle);

        this.setDesiredState(newState, isOpenLoop, true);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean suppressTurningAtLowSpeed) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mDesiredState = desiredState;

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kMaxSpeed;
            mDriveMotor.set(percentOutput);
        } else {
            mDriveController.setReference(
                desiredState.speedMetersPerSecond,
                ControlType.kVelocity, 0,
                mFeedforward.calculate(desiredState.speedMetersPerSecond)
            );
        }

		double angle = (suppressTurningAtLowSpeed && Math.abs(desiredState.speedMetersPerSecond) < kMaxSpeed * 0.01) ?
			mLastAngle :
			desiredState.angle.getRadians();

		mAngleController.setReference(angle, ControlType.kPosition);
		mLastAngle = angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        setDesiredState(desiredState, isOpenLoop, true);
    }

    public int getModuleNumber() {
        return mModuleNumber;
    }

    public SwerveModuleState getDesiredState() {
        return mDesiredState;
    }

    public void putSmartDashboardValues() {
        // SmartDashboard.putNumber("mod " + mModuleNumber + " encoder", mSteeringEncoder.getDistance());
        
		// SmartDashboard.putNumber("mod " + mModuleNumber + " absEnc.getDistance()", mSteeringEncoder.getDistance() - mAngleOffset);
		
		SmartDashboard.putNumber("mod " + mModuleNumber + " encoder absolute", mAngleEncoder.getAbsolutePosition());
		SmartDashboard.putNumber("mod " + mModuleNumber + " encoder relative", mIntegratedAngleEncoder.getPosition());

        // SmartDashboard.putNumber("mod " + mModuleNumber + " integrated", mIntegratedAngleEncoder.getPosition());
        // SmartDashboard.putNumber("mod " + mModuleNumber + " velocity", mDriveEncoder.getVelocity());

        // SmartDashboard.putNumber("mod " + mModuleNumber + " desired angle", mDesiredState.angle.getRadians());
    }

    public void resetToAbsolute() {
		mIntegratedAngleEncoder.setPosition(getShiftedAbsoluteEncoderRotation().getRadians());
    }

    public double getAbsoluteEncoderValue() {
        return mAngleEncoder.getAbsolutePosition();
    }

    public Rotation2d getAbsoluteEncoderRotation() {
        return Rotation2d.fromDegrees(getAbsoluteEncoderValue());
    }

    public Rotation2d getShiftedAbsoluteEncoderRotation() {
        return getAbsoluteEncoderRotation().minus(mAbsoluteOffset);
    }

    public double getRelativeEncoderValue() {
        return mIntegratedAngleEncoder.getPosition();
    }

    public Rotation2d getRelativeEncoderRotation() {
        return Rotation2d.fromRadians(getRelativeEncoderValue());
    }

    private void configureDriveMotor() {
        mDriveMotor.restoreFactoryDefaults();
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
        mAngleMotor.setInverted(Angle.kInverted);
        mIntegratedAngleEncoder.setPositionConversionFactor(Angle.kPositionConversionFactor);
        mAngleController.setP(Angle.kP);
        mAngleController.setI(Angle.kI);
        mAngleController.setD(Angle.kD);
        mAngleController.setFF(Angle.kFF);
        mAngleController.setPositionPIDWrappingMaxInput(Math.PI);
        mAngleController.setPositionPIDWrappingMinInput(-Math.PI);
        mAngleController.setPositionPIDWrappingEnabled(true);
        mAngleMotor.enableVoltageCompensation(kVoltageCompensation);
        mAngleMotor.burnFlash();
    }

    public SwerveModuleState getState() {
        double velocity = mDriveEncoder.getVelocity();
        Rotation2d angle = Rotation2d.fromRadians(mIntegratedAngleEncoder.getPosition());
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double drivePosition = mDriveEncoder.getPosition();
        Rotation2d angle = getState().angle;
        return new SwerveModulePosition(drivePosition, angle);
    }

    public void zeroPIDP() {
        mAngleController.setP(0);
    }
}