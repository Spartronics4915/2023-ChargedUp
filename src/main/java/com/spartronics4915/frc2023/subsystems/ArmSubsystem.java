// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.spartronics4915.frc2023.Constants.ArmConstants;
import com.spartronics4915.frc2023.Constants.ArmConstants.AbsoluteEncoderConstants;
import com.spartronics4915.frc2023.Constants.ArmConstants.PIDConstants;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax mWristMotor;
    private CANSparkMax mShoulderMotor;
    private AnalogEncoder mWristAbsEncoder;
    private AnalogEncoder mShoulderAbsEncoder;
    private SparkMaxPIDController mShoulderPIDContoller;
    private SparkMaxPIDController mWristPIDContoller;
    private double StartAngle = 20;
    /** Creates a new ExampleSubsystem. */
    public ArmSubsystem() {
        //motor setup
        mShoulderMotor = new CANSparkMax(ArmConstants.kShoulderMotorId,MotorType.kBrushed);
        mWristMotor = new CANSparkMax(ArmConstants.kWristMotorId,MotorType.kBrushed);
        //TODO check if motors are brushed or brushless, if brushed check if Idle Mode Setting is needed
        //note: seems there are going to be two motors for the shoulder maybe use the follow method
        mShoulderMotor.setIdleMode(IdleMode.kBrake); 
        mWristMotor.setIdleMode(IdleMode.kBrake); 
        // mWristAbsEncoder = new AnalogEncoder()
       

        mWristAbsEncoder = initializeAbsEncoder(ArmConstants.kWristAbsEncoder);
        mShoulderAbsEncoder = initializeAbsEncoder(ArmConstants.kShoulderAbsEncoder);

        
        //PID setup
        mShoulderPIDContoller = initializePIDController(mShoulderMotor, ArmConstants.kShoulderPID);
        mWristPIDContoller = initializePIDController(mWristMotor, ArmConstants.kWristPID);
        //TODO make sure to add absolute encoders
        //should create a way to test where if the joystick button is pressed the angle the motors go to is incremented by ten
        //TODO calculate gear ratios since the shoulder motors attach to a chain system to rotate the actual arm, because of this you can't use the follow method
        
    }

    private SparkMaxPIDController initializePIDController(CANSparkMax mMotor, PIDConstants kPIDConstants) {
        SparkMaxPIDController PIDController = mMotor.getPIDController();
        PIDController.setP(kPIDConstants.kP);
        PIDController.setI(kPIDConstants.kI);
        PIDController.setD(kPIDConstants.kD);
        PIDController.setReference(StartAngle, CANSparkMax.ControlType.kPosition, 0);
        return PIDController;
    }
    private AnalogEncoder initializeAbsEncoder(AbsoluteEncoderConstants kAbsoluteEncoderConstants) {
        AnalogEncoder absoluteEncoder = new AnalogEncoder(new AnalogInput(kAbsoluteEncoderConstants.channel));
        absoluteEncoder.setPositionOffset(kAbsoluteEncoderConstants.angleOffset);
        absoluteEncoder.setDistancePerRotation(2*Math.PI);
        return absoluteEncoder;
    }
    private void levelWrist(){
        //TODO add method which uses the shoulders setpoint to make the wrist stay level
    }
    
    public void setShoulderSetpoint(double value) {
        mShoulderPIDContoller.setReference(value, CANSparkMax.ControlType.kPosition);
    }
    public void setWristSetpoint(double value) {
        mWristPIDContoller.setReference(value, CANSparkMax.ControlType.kPosition);
    }
    
    /**
     * Example command factory method.
     *
     * @return a command
     */
    public CommandBase exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    //TODO should make a command that will pass it a class with the settings for the actuator and the shoulder motor angle

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //TODO maybe have the wrist encoder be offset to be an absolute value so you dont have to constantly re calculate it
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
