package com.spartronics4915.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.SparkMaxAlternateEncoder;
// import com.spartronics4915.frc2023.Constants.Arm;
import com.spartronics4915.frc2023.Constants.Arm.ExtenderConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtenderSubsystem extends SubsystemBase  {

    private final int kMotorID;
    private CANSparkMax mMotor;
    public RelativeEncoder mEncoder;
    private final double kRevPerInch = 12.0;
    public SparkMaxPIDController mPIDController;
    public double targetReference;

    private DigitalInput mLimitSwitchZero;


    // This is to work around a bug in the encoder class
    // The RelativeEncoder class cannot be negative. So we have to pad it out to 
    // a large value.
    private final double kPositionPad = 24;
    private final double kMinDist = 0.5;
    private final double kMaxDist = 12;
    private final double kPosTolerance = 0.2;
    private MotorAbsEncoderComboSubsystem mPivot;

    public ExtenderSubsystem(MotorAbsEncoderComboSubsystem pivot) {
        kMotorID = ExtenderConstants.kMotorID;
        mMotor = new CANSparkMax(kMotorID, MotorType.kBrushed);
        mEncoder = mMotor.getEncoder(Type.kQuadrature, 8192);

        mLimitSwitchZero = new DigitalInput(ExtenderConstants.kLimitSwitchZeroPort);


        //mEncoder = mMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        mMotor.restoreFactoryDefaults();
        mMotor.setInverted(true);
        mMotor.setSmartCurrentLimit(60);
        mPIDController = mMotor.getPIDController();
        mPIDController.setFeedbackDevice(mEncoder);
        mEncoder.setInverted(true);
        mEncoder.setPositionConversionFactor(1.0/kRevPerInch );// / kRevPerInch);
        mPIDController.setP(0.00007);
        mPIDController.setFF(0.00008);

        mEncoder.setPosition(kPositionPad);
        mPivot = pivot;
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
        System.out.println("Extending Position: " + getPosition());
        System.out.println("speed: " + mMotor.getAppliedOutput());

        mPIDController.setReference(90*60, ControlType.kVelocity);

        if(getPosition() >= kMaxDist) {
            mMotor.stopMotor();
        }
        else {

            // double extendSpeed = 0.5;

            // if(mPivot.getArmPosition().getDegrees()> -20)
            //     extendSpeed = 0.8;    
            // mMotor.set(extendSpeed);
        }

    }

    public void stopExtender() {
        mMotor.stopMotor();
    }

    public void startRetracting() {
        if(false){//(getPosition() < kMinDist){
            mMotor.stopMotor();
        }
        else {
            mPIDController.setReference(-90*60, ControlType.kVelocity);
            // double retractSpeed = -0.4;

            // if(mPivot.getArmPosition().getDegrees()< -20)
            //     retractSpeed = -0.8;    
            // mMotor.set(retractSpeed);
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
    }

    public boolean closeEnough() {
        return ((Math.abs(getPosition() - targetReference)) < kPosTolerance);
    }
    public double getReference() {

        return targetReference;
    }


    //maybe do this in periodic
    //if back limit switch is triggered, set pos to 0, 
        //if motor is going backwards (aka applied output is negative)
            //stop motor
    //create function to set encoder to value
    public void limitSwitchUpdate() {
        if (!mLimitSwitchZero.get()) {
            System.out.println("triggered");
            mEncoder.setPosition(0);
            if (mMotor.getAppliedOutput() < 0) {
                stopMotor();
            }
        }
    }

    @Override
    public void periodic() {
        limitSwitchUpdate();
    }
}

