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
// import com.spartronics4915.frc2023.Constants.ArmConstants.PIDConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorAbsEncoderComboSubsystem extends SubsystemBase {

    public interface AngleWithEarthProvider {
        Rotation2d getAngleWithEarth();
    }

    private CANSparkMax mMotor;
    private RelativeEncoder relEncoder;
    private SparkMaxAbsoluteEncoder mAbsEncoder;
    private SparkMaxPIDController mPIDController;
    private Rotation2d mCurrentReference = new Rotation2d(0);

    private boolean mActive;
    private boolean mReferenceSet;
    private double mReferenceRadians;
    private double mLastSpeedOutput;
    private AngleWithEarthProvider mAngleProvider;
    private double kP, kFF;
    private final TrapezoidProfile.Constraints motionConstraints;
    private double mModeledVelocity;

    public MotorAbsEncoderComboSubsystem(ArmMotorConstants MotorConstants, MotorType motorType) {

        motionConstraints = new TrapezoidProfile.Constraints(Math.PI *20/180, Math.PI *20/180*3);
        mModeledVelocity = 0;

        mMotor = new CANSparkMax(MotorConstants.kMotorID, motorType);
        mMotor.setIdleMode(IdleMode.kBrake);

        mAbsEncoder = mMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        mAbsEncoder.setPositionConversionFactor(Math.PI * 2);
        double radianOffset = MotorConstants.kZeroOffset.getRadians();
        if (radianOffset < 0) {
            radianOffset += Math.PI * 2;
        }
        mAbsEncoder.setZeroOffset(radianOffset);
        if (MotorConstants.kInvertMotor) {
            mMotor.setInverted(true);
        }
        mPIDController = null; //initializePIDController(MotorConstants);
        mMotor.setSmartCurrentLimit(20);
        mActive = true;
        mReferenceSet = false;
        mReferenceRadians = 0;
        mLastSpeedOutput = 0;
        mAngleProvider = null;

        kP = MotorConstants.kP;
        kFF = MotorConstants.kFF;
    }

    public void setAngleWithEarthProvider(AngleWithEarthProvider angleProvider) {
        mAngleProvider = angleProvider;
    }

    private SparkMaxPIDController initializePIDController(ArmMotorConstants MotorConstants) {
        SparkMaxPIDController PIDController = mMotor.getPIDController();
        PIDController.setP(MotorConstants.kP);
        PIDController.setI(MotorConstants.kI);
        PIDController.setD(MotorConstants.kD);

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

    public Rotation2d getCurrentReferenceArm() {
        return armToNative(mCurrentReference);
    }

    public Rotation2d getArmPosition() {
        return nativeToArm(getNativePosition());
    }

    public Rotation2d getNativePosition() {
        return Rotation2d.fromRadians(getRawPosition());
    }

    public double getRawPosition() {
        // System.out.println("get Pos Called");

            return mAbsEncoder.getPosition();
        // return 123;
    }

    public double getMotorSpeed() {
        return mMotor.getAppliedOutput();
    }

    public double getLastSpeedOutput() {
        return mLastSpeedOutput;
    }

    public void setMotor(double setting) {
        mMotor.set(setting);

    }

    public Rotation2d getVelocity() {
        return Rotation2d.fromRadians(mAbsEncoder.getVelocity());
    }

    @Override
    public void periodic() {

            double angleWithEarth;
            
            if(mAngleProvider==null) {

              angleWithEarth = getArmPosition().getRadians();
            }
            else {
                angleWithEarth = mAngleProvider.getAngleWithEarth().getRadians();
            }

            double currVelocity = getVelocity().getRadians();
            double currPosNative = getNativePosition().getRadians();

            var currState = new TrapezoidProfile.State(currPosNative, mModeledVelocity);
            var goalState = new TrapezoidProfile.State(mReferenceRadians, 0);
            TrapezoidProfile currMotionProfile = new TrapezoidProfile(motionConstraints, goalState, currState);
            double ticLength = 1./50; // Robot runs at 50Hz
            var state = currMotionProfile.calculate(ticLength);

            double pidReferenceRadians = state.position;
            mModeledVelocity = state.velocity;
            // currPosArm=nativeToArm(Rotation2d.fromDegrees(180)).getRadians();
            // currPosNative = Rotation2d.fromDegrees(180).getRadians();

            double ffComponent = -kFF * Math.cos(angleWithEarth);
            double err = kP*(pidReferenceRadians - currPosNative);

            double total_output = ffComponent + err;

            if(total_output > 1.0) {
                total_output = 1;
            } else if (total_output < -1.0) {
                total_output = -1.0;
            }

            if(mActive && mReferenceSet) {
                mMotor.set(total_output);
                mLastSpeedOutput = total_output;


        }
    }

}
