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
    private Rotation2d mLastReference;

    public MotorAbsEncoderComboSubsystem(int motorId, PIDConstants kPIDConstants) {
        mMotor = new CANSparkMax(motorId, MotorType.kBrushless);
        mMotor.setIdleMode(IdleMode.kCoast);
        mPIDController = initializePIDController(kPIDConstants);
    }

    private SparkMaxPIDController initializePIDController(PIDConstants kPIDConstants) {
        SparkMaxPIDController PIDController = mMotor.getPIDController();
        PIDController.setP(kPIDConstants.kP);
        PIDController.setI(kPIDConstants.kI);
        PIDController.setD(kPIDConstants.kD);
        PIDController.setPositionPIDWrappingEnabled(true);
        PIDController.setPositionPIDWrappingMaxInput(2*Math.PI);
        PIDController.setPositionPIDWrappingMinInput(0);
        
        return PIDController;
    }

    public void setReference(Rotation2d ref) {
        mLastReference = ref;
        mPIDController.setReference(ref.getDegrees(), ControlType.kPosition);
    }

}
