package com.spartronics4915.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.spartronics4915.frc2023.Constants.Arm.ArmMotorConstants;
// import com.spartronics4915.frc2023.Constants.ArmConstants.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    private final ArmMotorConstants motorConstants; 

    private boolean mActive;
    private boolean mReferenceSet;
    private double mReferenceRadians;
    private double mLastSpeedOutput;
    private AngleWithEarthProvider mAngleProvider;
    private double kP, kFF;
    private final TrapezoidProfile.Constraints motionConstraints;
    private double mModeledVelocity;
    public double trapezoidTarget;
    public double mModeledPosition;
    private static int instancecount =0;
    private int mycount;
    public MotorAbsEncoderComboSubsystem(ArmMotorConstants motorConstants, MotorType motorType) {
        this.motorConstants = motorConstants;
        mycount = instancecount;
        instancecount +=1;
        motionConstraints = new TrapezoidProfile.Constraints(Math.PI/6., Math.PI/6);
        mModeledVelocity = 0;

        mMotor = new CANSparkMax(motorConstants.kMotorConstants.kMotorID, motorType);
        mMotor.restoreFactoryDefaults();
        mMotor.setIdleMode(motorConstants.kMotorConstants.kIdleMode);

        mAbsEncoder = mMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        mAbsEncoder.setPositionConversionFactor(motorConstants.kMotorConstants.kPositionConversionFactor);

        double radianOffset = motorConstants.kMotorConstants.kZeroOffset.getRadians();

        if (radianOffset < 0) radianOffset += Math.PI * 2;

        mAbsEncoder.setZeroOffset(radianOffset);

        mMotor.setInverted(motorConstants.kMotorConstants.kInverted);

        mPIDController = null; //initializePIDController(MotorConstants);
        mMotor.setSmartCurrentLimit(20);
        mActive = true;
        mReferenceSet = false;
        mCurrentReference = getNativePosition();
        mReferenceRadians = mCurrentReference.getRadians();
        mLastSpeedOutput = 0;
        mAngleProvider = null;
        kP = motorConstants.kPIDConstants.kP;
        kFF = motorConstants.kPIDConstants.kFF;
        mMotor.burnFlash();
        
    }

    public void setAngleWithEarthProvider(AngleWithEarthProvider angleProvider) {
        mAngleProvider = angleProvider;
    }

    private SparkMaxPIDController initializePIDController() {
        SparkMaxPIDController pidController = mMotor.getPIDController();
        pidController.setP(motorConstants.kPIDConstants.kP);
        pidController.setI(motorConstants.kPIDConstants.kI);
        pidController.setD(motorConstants.kPIDConstants.kD);
        pidController.setFF(motorConstants.kPIDConstants.kFF);

        pidController.setSmartMotionMaxAccel(motorConstants.kSmartMotionsConstants.kSmartMotionMaxAccel, 0);
        pidController.setSmartMotionMaxVelocity(motorConstants.kSmartMotionsConstants.kSmartMotionMaxVelocity, 0);
        pidController.setSmartMotionMinOutputVelocity(motorConstants.kSmartMotionsConstants.kSmartMotionMinOutputVelocity, 0);

        pidController.setFeedbackDevice(mAbsEncoder);
        return pidController;
    }

    /**
     * Sets the reference.
     *
     * @param ref A Rotation2d. This should be in arm coordinates where 0 points
     *            directly out from the arm
     */
    public void setHorizonReference(Rotation2d ref) {
        // This is designed to ignore unsafe arm positions.
        if (Math.abs(ref.getDegrees()) > 80)
        {
            System.out.println("Unsafe arm position requested: " + ref);
            return;
        }
        mModeledPosition = getNativePosition().getRadians();
        setNativeReference(HorizonToNative(ref));
    }

    public void setActive(boolean active) {
        mActive = active;
    }

    public Rotation2d HorizonToNative(Rotation2d armRotation) {
        var outputRot = armRotation.unaryMinus().plus(motorConstants.kHorizonDeflection);

        // To match the encoder, we want angles in [0, 2PI]
        var radValue = outputRot.getRadians();
        if (radValue < 0) {
            outputRot = Rotation2d.fromRadians(radValue + 2 * Math.PI);
        }

        return outputRot;
    }

    public Rotation2d nativeToHorizon(Rotation2d nativeRotation) {
        return nativeRotation.minus(motorConstants.kHorizonDeflection).unaryMinus();
    }

    private void setNativeReference(Rotation2d ref) {

        System.out.println("SetNativeReferenceCalled");
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
        return nativeToHorizon(mCurrentReference);
    }

    public Rotation2d getHorizonPosition() {
        return nativeToHorizon(getNativePosition());
    }

    public Rotation2d getNativePosition() {
        return Rotation2d.fromRadians(mAbsEncoder.getPosition());
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

            double angleWithEarth = ((mAngleProvider == null) ?  getHorizonPosition() : mAngleProvider.getAngleWithEarth()).getRadians();

            // double currVelocity = getVelocity().getRadians();
            double currPosNative = getNativePosition().getRadians();

            // var currState = new TrapezoidProfile.State(mModeledPosition, mModeledVelocity);
            // var goalState = new TrapezoidProfile.State(mReferenceRadians, 0);
            // TrapezoidProfile currMotionProfile = new TrapezoidProfile(motionConstraints, goalState, currState);
            // double ticLength = 1./50; // Robot runs at 50Hz
            // var state = currMotionProfile.calculate(ticLength);

            // // Trapezoid profiling not working, so this effectively disables it.
            
            double pidReferenceRadians = mReferenceRadians;//state.position;
            //trapezoidTarget = state.position;

            //mModeledVelocity = state.velocity;
            // By doing this, we are just playing back the motion profile in case the PID controller doesn't keep up.
            //mModeledPosition = state.position;
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
