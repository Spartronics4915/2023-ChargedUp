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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Understanding native and arm coordinate systems:
 * To reason about motions, it is most conventient to put 0 degrees at horizontal and use CCW coordinates
 * So +30 is up and -30 is down.  But, this puts wrap-around right at horizontal.
 * These are called "arm coordinates".
 * To avoid having to handle wraparound everywhere, the encoder is actually set so that horizontal is at 180.
 * This gives 180 degrees up and down and avoids having to think about handling warp-around
 * These are "native coordinates", the coordinates of the actual encoder
 * They should be strictly internal and all other systems should use arm coordinates.
 */
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
    public double trapezoidTarget;
    public double pidErr;
    public double mModeledPosition;
    private final Rotation2d mMaxRotation, mMinRotation;
    public boolean isArm;

    public MotorAbsEncoderComboSubsystem(ArmMotorConstants MotorConstants, MotorType motorType) {

        isArm = (mAngleProvider==null);

        motionConstraints = new TrapezoidProfile.Constraints(MotorConstants.kMaxVelocity, MotorConstants.kMaxAccel);
        mModeledVelocity = 0;

        mMotor = new CANSparkMax(MotorConstants.kMotorID, motorType);
        mMotor.restoreFactoryDefaults();
        mMotor.setIdleMode(IdleMode.kBrake);

        mAbsEncoder = mMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        mAbsEncoder.setPositionConversionFactor(Math.PI * 2);
        double radianOffset = MotorConstants.kZeroOffset.getRadians();
        if (radianOffset < 0) {
            radianOffset += Math.PI * 2;
        }
        mAbsEncoder.setZeroOffset(radianOffset);
        if((mAbsEncoder.getPosition() == 0) && MotorConstants.kZeroOffset.getDegrees() > 0) {
            mActive = false;
            System.out.println("zero offset not applied, disabling");
            SmartDashboard.putBoolean("disabled", true);
        } else mActive = true;
        if (MotorConstants.kInvertMotor) {
            mMotor.setInverted(true);
        }
        mPIDController = null; //initializePIDController(MotorConstants);
        mMotor.setSmartCurrentLimit(40);
        mActive = true;
        mReferenceSet = false;

        makeModeledPositionsMatchPhysical();

        mLastSpeedOutput = 0;
        mAngleProvider = null;
        kP = MotorConstants.kP;
        kFF = MotorConstants.kFF;
        mMotor.burnFlash();

        mMaxRotation = MotorConstants.mMaxRotation;
        mMinRotation = MotorConstants.mMinRotation;

    }

    public void makeModeledPositionsMatchPhysical() {
        mCurrentReference = getNativePosition();
        mModeledPosition = getNativePosition().getRadians();
        mReferenceRadians = mCurrentReference.getRadians();
        mModeledVelocity = 0;
    }

    public Rotation2d getModeledPosition() {
        return nativeToArm(Rotation2d.fromRadians(mModeledPosition));
    }

    public void setAngleWithEarthProvider(AngleWithEarthProvider angleProvider) {
        mAngleProvider = angleProvider;
        isArm = (mAngleProvider==null);

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
        // This is designed to ignore unsafe arm positions.
        if (ref.getDegrees() > mMaxRotation.getDegrees())
        {
            System.out.println("Arm:" + isArm +" Unsafe arm position requested (setArmReference): " + ref);
            return;
        }
        Rotation2d nativeRef = armToNative(ref);
       // System.out.println("Arm:" + isArm +" setArmReference: " + nativeRef.getDegrees() + " " + ref.getDegrees());
        setNativeReference(nativeRef);
    }

    public void clearReference() {
        mReferenceSet = false;
        this.stopMotor();

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

        if (false){//{(Math.abs(mCurrentReference.minus(ref).getDegrees()) > 120) {
            System.out.println("setNativeReference illegal request:" + ref);
            return;
        }

        Rotation2d refArm = nativeToArm(ref);
        if(refArm.getDegrees() > mMaxRotation.getDegrees()) {
            System.out.println("Arm:" + isArm +" Requested ref " + refArm + " greater than max " +mMaxRotation);
            return;
        } else
        if(refArm.getDegrees() < mMinRotation.getDegrees()) {
            System.out.println("Requested ref " + refArm + " less than min " +mMaxRotation);
            return;
        }
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
        return nativeToArm(mCurrentReference);
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

    public void stopMotor() {
        mReferenceSet = false;
        mMotor.stopMotor();
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

            // double currVelocity = getVelocity().getRadians();
            double currPosNative = getNativePosition().getRadians();

            var currState = new TrapezoidProfile.State(mModeledPosition, mModeledVelocity);
            var goalState = new TrapezoidProfile.State(mReferenceRadians, 0);
            TrapezoidProfile currMotionProfile = new TrapezoidProfile(motionConstraints, goalState, currState);
            double ticLength = 1./50; // Robot runs at 50Hz
            var state = currMotionProfile.calculate(ticLength);
           
            // double pidReferenceRadians = mReferenceRadians;//state.position;
            double pidReferenceRadians = state.position;
            trapezoidTarget = state.position;

            mModeledVelocity = state.velocity;
            // By doing this, we are just playing back the motion profile in case the PID controller doesn't keep up.
            mModeledPosition = state.position;

            double ffComponent = -kFF * (Math.copySign(0.1, Math.cos(angleWithEarth)) + Math.cos(angleWithEarth));
            pidErr = (pidReferenceRadians - currPosNative);
            
            double total_output = ffComponent + kP*pidErr;

            double maxSpeed = 0.4;

            if(total_output > maxSpeed) {
                total_output = maxSpeed;
            } else if (total_output < -maxSpeed) {
                total_output = -maxSpeed;
            }

            if(mActive && mReferenceSet) {
                mMotor.set(total_output);
                mLastSpeedOutput = total_output;
                // if(isArm) {
                // System.out.println("Setting motor " + total_output + " " + mMotor.getAppliedOutput() + " " + "modeledPos: " + mModeledPosition + "mRef " + mCurrentReference );
                // }
            } else {
                this.makeModeledPositionsMatchPhysical();
                stopMotor();
            }
    }

}
