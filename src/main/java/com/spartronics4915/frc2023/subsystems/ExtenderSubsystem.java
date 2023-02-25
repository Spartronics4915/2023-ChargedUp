package com.spartronics4915.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtenderSubsystem extends SubsystemBase  {

    private final int kMotorID;
    private CANSparkMax mMotor;
    private RelativeEncoder mEncoder;
    private final double kRevPerInch = 12.0;
    //private SparkMaxPIDController mPIDController;
    public double targetReference;

    // This is to work around a bug in the encoder class
    // The RelativeEncoder class cannot be negative. So we have to pad it out to 
    // a large value.
    private final double kPositionPad = 24;
    private final double kMinDist = 0.5;
    private final double kMaxDist = 12;
    private final double kPosTolerance = 0.2;

    public ExtenderSubsystem(int motorID) {
        kMotorID = motorID;
        mMotor = new CANSparkMax(kMotorID, MotorType.kBrushed);
        mEncoder = mMotor.getEncoder(Type.kQuadrature, 8192);
        //mEncoder = mMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        mMotor.restoreFactoryDefaults();
        mMotor.setInverted(true);
        mMotor.setSmartCurrentLimit(10);
        //mPIDController = mMotor.getPIDController();
        //mPIDController.setFeedbackDevice(mEncoder);
        mEncoder.setInverted(true);
        mEncoder.setPositionConversionFactor(1.0/kRevPerInch );// / kRevPerInch);
        //mPIDController.setP(1./60);

        mEncoder.setPosition(0);
        targetReference = 0;
    }

    public void setZero() {
        mEncoder.setPosition(kPositionPad);

    }

    public void stopMotor() {
        mMotor.stopMotor();
    }
    
    public double getPosition() {
        return (getRawPosition() - kPositionPad);
    }

    public double getRawPosition() {
        return mEncoder.getPosition();
    }

    public SparkMaxPIDController getPIDController() {

        return null;//mPIDController;
    }

    public CANSparkMax getMotor() {

        return mMotor;
    }
    
    public void startExtending() {

        if(getPosition() >= kMaxDist) {
            mMotor.stopMotor();
        }
        else {
            mMotor.set(0.3);
        }

    }

    public void stopExtender() {
        mMotor.stopMotor();
    }

    public CommandBase getExtendCommand() {

        return Commands.runEnd(() -> this.startExtending(), () -> this.stopMotor());
    }

    public CommandBase getRetractCommand() {
        return Commands.runEnd(() -> this.startRetracting(), () -> this.stopMotor());
    }

    public void startRetracting() {
        if(getPosition() < kMinDist) {
            mMotor.stopMotor();
        }
        else {
            mMotor.set(-0.3);
        }
    }

    public boolean atPos(double pos) {
        return (Math.abs(getPosition() - pos) < kPosTolerance);
    }

    public CommandBase extendToNInches(double N) {
        return Commands.runEnd(()->this.startExtending(), () -> this.stopMotor()).until(() -> atPos(N));
    }

    public void setReference(double p) {
        targetReference = p;
        //mPIDController.setReference(p, ControlType.kPosition);
    }

    public boolean closeEnough() {
        return ((Math.abs(getPosition() - targetReference)) < 0.1);
    }
    public double getReference() {

        return targetReference;
    }

}
