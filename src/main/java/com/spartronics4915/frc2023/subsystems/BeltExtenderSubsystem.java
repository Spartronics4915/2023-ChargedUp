package com.spartronics4915.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.SparkMaxAlternateEncoder;
// import com.spartronics4915.frc2023.Constants.Arm;
import com.spartronics4915.frc2023.Constants.Arm.ExtenderConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class BeltExtenderSubsystem extends SubsystemBase  {

    private final int kMotorID;
    private CANSparkMax mMotor;
    public RelativeEncoder mEncoder;
    private final double kRevPerInch = 16.5 / 6.75;
    public SparkMaxPIDController mPIDController;

    private DigitalInput mLimitSwitchZero;
    private boolean mIsActive;

    // This is to work around a bug in the encoder class
    // The RelativeEncoder class cannot be negative. So we have to pad it out to 
    // a large value.
    private final double kPositionPad = 24;
    private final double kMinDist = 0.5;
    private final double kMaxDist = 16.5;
    private final double kPosTolerance = 0.2;
    private MotorAbsEncoderComboSubsystem mPivot;
    private TrapezoidProfile.Constraints motionConstraints;
    private TrapezoidProfile.State currModeledState, targetState;

    public BeltExtenderSubsystem(MotorAbsEncoderComboSubsystem pivot) {
        kMotorID = ExtenderConstants.kMotorID;
        mMotor = new CANSparkMax(kMotorID, MotorType.kBrushless);
        mMotor.restoreFactoryDefaults();
        mEncoder = mMotor.getEncoder();

        mLimitSwitchZero = new DigitalInput(ExtenderConstants.kLimitSwitchZeroPort);


        //mEncoder = mMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        mMotor.restoreFactoryDefaults();

        mMotor.setIdleMode(IdleMode.kCoast);
        
        mMotor.setSmartCurrentLimit(40);
        
        mPIDController = mMotor.getPIDController();
        mEncoder.setPositionConversionFactor(1.0/kRevPerInch );// / kRevPerInch);
        mPIDController.setP(1.5);
        mPIDController.setFF(0.0000);
        mEncoder.setPosition(kPositionPad);
        mPivot = pivot;

        mIsActive = true;

        motionConstraints = new TrapezoidProfile.Constraints(20, 30);
        currModeledState = new TrapezoidProfile.State(kPositionPad, 0);
        targetState = new TrapezoidProfile.State(currModeledState.position, 0);

        mPIDController.setOutputRange(-0.6, 0.6);
        mMotor.burnFlash();
    }

    public void makeModeledPositionsMatchPhysical() {

        currModeledState = new TrapezoidProfile.State(getRawPosition(), 0);
        targetState = new TrapezoidProfile.State(getRawPosition(), 0);
    }


    public boolean isActive() {
        return mIsActive;
    }

    public void setIsActive(boolean active) {
        mIsActive = active;
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

        return mPIDController;
    }

    public void setTarget(double target) {

        if(target > kMaxDist) {
            target = kMaxDist;
        }

        double targetReference = target + kPositionPad;
        targetState = new TrapezoidProfile.State(targetReference, 0);
    }

    public double getTarget() {
        return targetState.position - kPositionPad;
    }

    public double getModeledPosition() {
        return currModeledState.position - kPositionPad;
    }

    public double getRawModeledPosition() {
        return currModeledState.position;
    }

    public CANSparkMax getMotor() {

        return mMotor;
    }
    

    public void stopExtenderAndMatchTargetToPhysical() {
        stopMotor();
        makeModeledPositionsMatchPhysical();
    }


    public boolean atPos(double pos) {
        return (Math.abs(getPosition() - pos) < kPosTolerance);
    }


    public boolean limitSwitchActive() {
        return !mLimitSwitchZero.get();
    };


    public void setZero() {
        mEncoder.setPosition(kPositionPad);
        if(getModeledPosition() < 0) {
            currModeledState = new TrapezoidProfile.State(kPositionPad, 0);
        }
    }

    public void modifyTarget(double delta) {
        double currTarget = getTarget();
        double newTarget = currTarget + delta;
        setTarget(newTarget);

    }

    public CommandBase modifyTargetCommandRunOnce(double delta) {

        var command = runOnce(()->modifyTarget(delta));

        return command;

    }
    public CommandBase modifyTargetCommandRepeat(double delta) {

        var command = run(()->modifyTarget(delta));

        return command;

    }

    public CommandBase setTargetCommandRunOnce(double target) {
        var command = runOnce(()->setTarget(target));

        return command;
    }

    public boolean modeledExtenderCloseEnoughToTarget() {
        return (Math.abs(getModeledPosition() - getTarget()) <= Math.abs(kPosTolerance));
    }

    public CommandBase waitForModeledExtenderToArriveCommand() {

        // This runs a no-op command and adds an until statement to it.
        return  this.run(()->{}).until(()->modeledExtenderCloseEnoughToTarget());

    }

    public CommandBase zeroExtenderCommand() {
        return new CommandBase() {
            @Override
            public void execute() {
                setTarget(-20);
            }

            @Override
            public boolean isFinished() {
                return !mLimitSwitchZero.get();
            }
        };
    }

    @Override
    public void periodic() {

        if (!mLimitSwitchZero.get()) {
            setZero();
            if (getTarget() <= 0) {
                setTarget(0);
            }
        } 
          
        final double ticLength = 1. / 50;
        TrapezoidProfile currMotionProfile = new TrapezoidProfile(motionConstraints, targetState, currModeledState);
        currModeledState = currMotionProfile.calculate(ticLength);
        mPIDController.setReference(currModeledState.position, ControlType.kPosition);
    }

}

