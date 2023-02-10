package com.spartronics4915.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.spartronics4915.frc2023.Constants.ArmConstants.PIDConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorAbsEncoderComboSubsystem extends SubsystemBase{
    
    private CANSparkMax mMotor;
    private SparkMaxAbsoluteEncoder mAbsEncoder;
    private SparkMaxPIDController mPIDContoller;

    public MotorAbsEncoderComboSubsystem(int motorId) {
        mMotor = new CANSparkMax(motorId, MotorType.kBrushless);
        mMotor.setIdleMode(IdleMode.kCoast); 
    }

    private void initializePIDController(PIDConstants kPIDConstants) {
        SparkMaxPIDController mPIDController = mMotor.getPIDController();
        mPIDController.setP(kPIDConstants.kP);
        mPIDController.setI(kPIDConstants.kI);
        mPIDController.setD(kPIDConstants.kD);
        mPIDController.setPositionPIDWrappingEnabled(true);
        mPIDController.setPositionPIDWrappingMaxInput(2*Math.PI);
        mPIDController.setPositionPIDWrappingMinInput(0);
    }

}
