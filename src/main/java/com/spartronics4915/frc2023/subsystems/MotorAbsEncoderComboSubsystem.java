package com.spartronics4915.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.spartronics4915.frc2023.Constants.Arm.ArmMotorConstants;
import com.spartronics4915.frc2023.Constants.ArmConstants.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorAbsEncoderComboSubsystem extends SubsystemBase {

    private CANSparkMax mMotor;
    private RelativeEncoder relEncoder;
    private SparkMaxAbsoluteEncoder mAbsEncoder;
    private SparkMaxPIDController mPIDController;
    private Rotation2d mCurrentReference = new Rotation2d(0);

    private boolean mActive;
    private boolean mReferenceSet;
    private double mReferenceRadians;
    private ArmMotorConstants constants;

    public MotorAbsEncoderComboSubsystem(ArmMotorConstants motorConstants, MotorType motorType) {
        constants = motorConstants;

        mMotor = new CANSparkMax(constants.kMotorID, motorType);
        mMotor.setIdleMode(IdleMode.kCoast);

        mAbsEncoder = mMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        mAbsEncoder.setPositionConversionFactor(constants.kPositionConversionFactor);
        
        if (constants.kInvertMotor) {
            mMotor.setInverted(true);
        }
        mPIDController = initializePIDController();
        mMotor.setSmartCurrentLimit(20);
        mActive = true;
        mReferenceSet = false;
        mReferenceRadians = 0;
    }

    private SparkMaxPIDController initializePIDController() {
        SparkMaxPIDController PIDController = mMotor.getPIDController();
        PIDController.setP(constants.kP);
        PIDController.setI(constants.kI);
        PIDController.setD(constants.kD);
        
        mAbsEncoder = mMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        mAbsEncoder.setPositionConversionFactor(constants.kPositionConversionFactor);    
        double radianOffset = constants.kZeroOffset.getRadians();
        if (radianOffset < 0) {
            radianOffset += Math.PI*2;
        }
        
        mAbsEncoder.setZeroOffset(radianOffset);
        PIDController.setFeedbackDevice(mAbsEncoder);
        return PIDController;
    }

    /**
     * Sets the reference.
     *
     * @param ref A Rotation2d. This should be in arm coordinates where 0 points
     *            directly out from the arm
     */
    public void setArmReference(Rotation2d ref) {
        setNativeReference(armToNative(ref));
    }

    public void setActive(boolean active) {
        mActive = active;
    }

    public Rotation2d armToNative(Rotation2d armRotation) {
        var outputRot = armRotation.unaryMinus().plus(Rotation2d.fromDegrees(180));

        // To match the encoder, we want angles in [0, 2PI]
        var radValue = outputRot.getRadians();
        if (radValue < 0) {
            outputRot = Rotation2d.fromRadians(radValue + 2 * Math.PI);
        }

        return outputRot;
    }

    public Rotation2d nativeToArm(Rotation2d nativeRotation) {
        return nativeRotation.minus(Rotation2d.fromDegrees(180)).unaryMinus();
    }

    private void setNativeReference(Rotation2d ref) {

        mCurrentReference = ref;
        mReferenceRadians = ref.getRadians();
        mReferenceSet = true;
    }

    public void resetToZero() {
        mAbsEncoder.setZeroOffset(-mAbsEncoder.getPosition());
    }

    public CANSparkMax getMotor() {
        return mMotor;
    }

    public Rotation2d getCurrentReference() {
        return mCurrentReference;
    }

    public Rotation2d getArmPosition() {
        return nativeToArm(getNativePosition());
    }

    public Rotation2d getNativePosition() {
        return Rotation2d.fromRadians(getRawPosition());
    }

    public double getRawPosition() {
        // System.out.println("get Pos Called");
            return (mMotor.getEncoder().getPosition());
        // return 123;
    }

    public double getMotorSpeed() {
        return mMotor.getAppliedOutput();
    }

    @Override
    public void periodic() {

        System.out.println(mActive + " " + mReferenceSet + " " + (mActive && mReferenceSet));
        double currPos = getArmPosition().getRadians();
        // final double kFF = 0.3;
        // final double kP = 0.2;
        double ffComponent = constants.kFF * Math.cos(currPos);
        mPIDController.setFF(ffComponent);
        // double err = kP*(mReferenceRadians - currPos);

        // double total_output = ffComponent + err;

        // if(total_output > 1.0) {
        //     total_output = 1;
        // } else if (total_output < -1.0) {
        //     total_output = -1.0;
        // }

        // System.out.println("Periodic Called " + total_output + " " + err + " " + ffComponent);
        // if(mActive && mReferenceSet) {
        //     mMotor.set(total_output);
        // }
    }

}
