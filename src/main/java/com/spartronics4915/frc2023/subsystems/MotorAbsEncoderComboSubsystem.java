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
    private Rotation2d mLastReference = new Rotation2d(0);
    private boolean useAbs;

    public MotorAbsEncoderComboSubsystem(ArmMotorConstants MotorConstants, boolean useAbs) {
        this.useAbs = useAbs;
        
        mMotor = new CANSparkMax(MotorConstants.kMotorID, MotorType.kBrushless);
        mMotor.setIdleMode(IdleMode.kCoast);

        if(useAbs){
            mAbsEncoder = mMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
            mAbsEncoder.setPositionConversionFactor(MotorConstants.kPositionConversionFactor);    
            mAbsEncoder.setZeroOffset(MotorConstants.kZeroOffset);
        } else {
            relEncoder = mMotor.getEncoder();
            relEncoder.setPositionConversionFactor(MotorConstants.kPositionConversionFactor);
        }

        mPIDController = initializePIDController(MotorConstants);
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
        PIDController.setPositionPIDWrappingEnabled(false);
        PIDController.setPositionPIDWrappingMaxInput(1);
        PIDController.setPositionPIDWrappingMinInput(0);
        if(useAbs) PIDController.setFeedbackDevice(mAbsEncoder);
        else {
            PIDController.setFeedbackDevice(mMotor.getEncoder());
        }
        System.out.println(PIDController.getP());
        System.out.println(PIDController.getI());
        System.out.println(PIDController.getD());
        return PIDController;
    }

    public void setReference(Rotation2d ref) {
        System.out.println("set Ref called");

        mLastReference = ref;
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
        return mLastReference;
    }

    public double getPosition(){
        // System.out.println("get Pos Called");
        if (useAbs) return mAbsEncoder.getPosition();
        else return(mMotor.getEncoder().getPosition());
        // return 123;
    }

    public double getMotorSpeed(){
        return mMotor.getAppliedOutput();
    }
}
