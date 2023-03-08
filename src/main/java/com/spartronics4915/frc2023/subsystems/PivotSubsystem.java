package com.spartronics4915.frc2023.subsystems;

import com.spartronics4915.frc2023.Constants.Arm.ArmMotorConstants;
import com.spartronics4915.frc2023.Constants.Arm.CanSparkMaxMotorConstants;
import com.spartronics4915.frc2023.Constants.Arm;

import com.revrobotics.CANSparkMax;

import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
    private CANSparkMax mMotor;
    private SparkMaxAbsoluteEncoder mAbsEncoder;
    private SparkMaxPIDController mPIDController;
    private final ArmMotorConstants kConstants;
    private Rotation2d currentRefrence;
    private boolean setRef;
    
    //important notes: the mpidcontroller can only work in native units

    public PivotSubsystem() {
        super();
        kConstants = Arm.kPivotMotorConstants;
        motorInit();
        encoderInit();
        pidControllerInit();
        setRef = false;
        currentRefrence = new Rotation2d(0);
    }

    //individual init methods:

    private void motorInit() {
        CanSparkMaxMotorConstants motorConstants =  kConstants.kMotorConstants;
        mMotor = new CANSparkMax(motorConstants.kMotorID, motorConstants.kMotorType);
        mMotor.setIdleMode(motorConstants.kIdleMode);
        mMotor.setInverted(motorConstants.kInverted);

        mMotor.setSmartCurrentLimit(20);
    }
    
    private void encoderInit() {
        CanSparkMaxMotorConstants motorConstants =  kConstants.kMotorConstants;
        mAbsEncoder = mMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        mAbsEncoder.setZeroOffset(motorConstants.kZeroOffset.getRotations()); //this is in rotation because the encoder's position conversion factor hasn't been set yet
        mAbsEncoder.setPositionConversionFactor(motorConstants.kPositionConversionFactor);
        mAbsEncoder.setInverted(motorConstants.kInverted);
    }

    private void pidControllerInit() {
        mPIDController = mMotor.getPIDController(); 
        
        mPIDController.setFeedbackDevice(mAbsEncoder);
        
        mPIDController.setP(kConstants.kPIDConstants.kP);
        mPIDController.setI(kConstants.kPIDConstants.kI);
        mPIDController.setD(kConstants.kPIDConstants.kD);
        mPIDController.setFF(kConstants.kPIDConstants.kFF);

        mPIDController.setSmartMotionMaxAccel(kConstants.kSmartMotionsConstants.kSmartMotionMaxAccel, 0);
        mPIDController.setSmartMotionMaxVelocity(kConstants.kSmartMotionsConstants.kSmartMotionMaxVelocity, 0);
        mPIDController.setSmartMotionMinOutputVelocity(kConstants.kSmartMotionsConstants.kSmartMotionMinOutputVelocity, 0);
    }

    //unit conversions:

    public Rotation2d HorizonToNative(Rotation2d armRotation) {
        var outputRot = armRotation.unaryMinus().plus(kConstants.kHorizonDeflection);

        // To match the encoder, we want angles in [0, 2PI]
        var radValue = outputRot.getRadians();
        if (radValue < 0) {
            outputRot = Rotation2d.fromRadians(radValue + 2 * Math.PI);
        }

        return outputRot;
    }

    public Rotation2d nativeToHorizon(Rotation2d nativeRotation) {
        return nativeRotation.minus(kConstants.kHorizonDeflection).unaryMinus();
    }
    
    //getters for position
    public Rotation2d getHorizonPosition() {
        return nativeToHorizon(getNativePosition());
    }

    public Rotation2d getNativePosition() {
        return Rotation2d.fromRadians(mAbsEncoder.getPosition());
    }

    //setters for position:
    public void setHorizonReference(Rotation2d ref) {
        // This is designed to ignore unsafe arm positions.
        setNativeReference(HorizonToNative(ref));
    }

    private void setNativeReference(Rotation2d ref) {
        currentRefrence = ref;
        setRef = true;
        // mPIDController.setReference(ref.getRadians(), ControlType.kSmartMotion, 0, Math.cos(getHorizonPosition().getRadians()));
    }

    //feedforward update
    @Override
    public void periodic() {
        super.periodic();
        if (setRef)
            mPIDController.setReference(
                currentRefrence.getRadians(), 
                ControlType.kSmartMotion, 0, 
                Math.cos(getHorizonPosition().getRadians())
            );

    }

    //getters for debug teleop:
    public CANSparkMax getMotor() {
        return mMotor;
    }

    public SparkMaxAbsoluteEncoder getAbsEncoder() {
        return mAbsEncoder;
    }

    /**
     * @return the refrence the PID controller is in horizon angles
     */
    public Rotation2d getCurrentRefrence() {
        return currentRefrence;
    }

    
}
