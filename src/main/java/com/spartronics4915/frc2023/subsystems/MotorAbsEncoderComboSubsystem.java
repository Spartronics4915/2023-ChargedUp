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

public class MotorAbsEncoderComboSubsystem extends SubsystemBase{
    
    private CANSparkMax mMotor;
    private RelativeEncoder relEncoder;
    private SparkMaxAbsoluteEncoder mAbsEncoder;
    private SparkMaxPIDController mPIDController;
    private Rotation2d mCurrentReference = new Rotation2d(0);
    private boolean useAbs;

    public MotorAbsEncoderComboSubsystem(ArmMotorConstants MotorConstants, boolean useAbs, MotorType motorType) {
        this.useAbs = useAbs;
        
        mMotor = new CANSparkMax(MotorConstants.kMotorID, motorType);
        mMotor.setIdleMode(IdleMode.kCoast);

        if(useAbs){
            mAbsEncoder = mMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
            mAbsEncoder.setPositionConversionFactor(MotorConstants.kPositionConversionFactor);    
            double radianOffset = MotorConstants.kZeroOffset.getRadians();
            if (radianOffset < 0) {
                radianOffset += Math.PI*2;
            }
            mAbsEncoder.setZeroOffset(radianOffset);
        } else {
            relEncoder = mMotor.getEncoder();
            relEncoder.setPositionConversionFactor(MotorConstants.kPositionConversionFactor);
        }
        if(MotorConstants.kInvertMotor) {
            mMotor.setInverted(true);
        }
        mPIDController = initializePIDController(MotorConstants);
        mMotor.setSmartCurrentLimit(20);
    }

    private SparkMaxPIDController initializePIDController(ArmMotorConstants MotorConstants) {
        SparkMaxPIDController PIDController = mMotor.getPIDController();
        PIDController.setP(MotorConstants.kP);
        PIDController.setI(MotorConstants.kI);
        PIDController.setD(MotorConstants.kD);

        PIDController.setSmartMotionMaxAccel(MotorConstants.kSmartMotionMaxAccel, 0); //20
 
        PIDController.setSmartMotionMaxVelocity(MotorConstants.kSmartMotionMaxVelocity, 0);

        PIDController.setSmartMotionMinOutputVelocity(MotorConstants.kSmartMotionMinOutputVelocity, 0);
        
        PIDController.setSmartMotionAllowedClosedLoopError(Rotation2d.fromDegrees(5).getRotations(), 0);
        // PIDController.setOutputRange(0, kP)
        PIDController.setPositionPIDWrappingEnabled(true);
        PIDController.setPositionPIDWrappingMaxInput(Math.PI);
        PIDController.setPositionPIDWrappingMinInput(-Math.PI);
        if(useAbs) PIDController.setFeedbackDevice(mAbsEncoder);
        else {
            PIDController.setFeedbackDevice(mMotor.getEncoder());
        }
        System.out.println(PIDController.getP());
        System.out.println(PIDController.getI());
        System.out.println(PIDController.getD());
        return PIDController;
    }

/**
   * Sets the reference.
   *
   * @param ref A Rotation2d.  This should be in arm coordinates where 0 points directly out from the arm
*/  
    public void setReference(Rotation2d ref) {
        setNativeReference(armToNative(ref));
    }

    public Rotation2d armToNative(Rotation2d armRotation) {
        var outputRot = armRotation.unaryMinus().plus(Rotation2d.fromDegrees(180));

        // To match the encoder, we want angles in [0, 2PI]
        var radValue = outputRot.getRadians();
        if (radValue < 0) {
            outputRot = Rotation2d.fromRadians(radValue+2*Math.PI);
        }

        return outputRot;
    }

    public Rotation2d nativeToArm(Rotation2d nativeRotation) {
        return nativeRotation.minus(Rotation2d.fromDegrees(180)).unaryMinus();
    }

    private void setNativeReference(Rotation2d ref) {
        System.out.println("set Ref called");

        mCurrentReference = ref;
        System.out.println(ref + ": also this was called");
        mPIDController.setReference(ref.getRadians(), ControlType.kSmartMotion);
    }
    public void resetToZero(){
        mAbsEncoder.setZeroOffset(-mAbsEncoder.getPosition());
    }

    public CANSparkMax getMotor(){
        return mMotor;
    }
    public Rotation2d getCurrentReference() 
    {
        return mCurrentReference;
    }

    public Rotation2d getArmPosition(){
        return nativeToArm(getNativePosition());
    }

    public Rotation2d getNativePosition(){
        return Rotation2d.fromRadians(getRawPosition());
    }
    
    public double getRawPosition(){
        // System.out.println("get Pos Called");
        if (useAbs) return mAbsEncoder.getPosition();
        else return(mMotor.getEncoder().getPosition());
        // return 123;
    }

    public double getMotorSpeed(){
        return mMotor.getAppliedOutput();
    }
    
}
