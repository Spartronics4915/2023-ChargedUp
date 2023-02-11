package com.spartronics4915.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.spartronics4915.frc2023.Constants.ArmConstants.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorAbsEncoderComboSubsystem extends SubsystemBase{
    
    private CANSparkMax mMotor;
    private SparkMaxAbsoluteEncoder mAbsEncoder;
    private SparkMaxPIDController mPIDController;
    private Rotation2d mLastReference = new Rotation2d(Math.PI*10);

    public MotorAbsEncoderComboSubsystem(int motorId, double kP) {
        mMotor = new CANSparkMax(motorId, MotorType.kBrushless);
        mMotor.setIdleMode(IdleMode.kCoast);

        mAbsEncoder = mMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        mAbsEncoder.setPositionConversionFactor(Math.PI*2);    
        
        mPIDController = initializePIDController(kP);
    }

    private SparkMaxPIDController initializePIDController(double kP) {
        SparkMaxPIDController PIDController = mMotor.getPIDController();
        PIDController.setP(kP);
        PIDController.setI(0);
        PIDController.setD(0);
        
        PIDController.setPositionPIDWrappingEnabled(true);
        PIDController.setPositionPIDWrappingMaxInput(2*Math.PI);
        PIDController.setPositionPIDWrappingMinInput(0);
        // PIDController.setFeedbackDevice(mAbsEncoder);
        
        return PIDController;
    }

    public void setReference(Rotation2d ref) {
        mLastReference = ref;
        System.out.println(ref + ": also this was called");
        mPIDController.setReference(ref.getRadians(), ControlType.kPosition);
    }

    public Rotation2d getCurrentReference() 
    {
        return mLastReference;
    }

    public double getPosition(){
        return mAbsEncoder.getPosition();
    }
    public double getMotorSpeed(){
        return mMotor.get();
    }
}
