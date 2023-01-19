// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.spartronics4915.frc2023.Constants.ArmConstants;
import com.spartronics4915.frc2023.Constants.ArmConstants.PIDConstants;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax mWristMotor;
    private CANSparkMax mShoulderMotor;
    private SparkMaxPIDController mShoulderPIDContoller;
    private SparkMaxPIDController mWristPIDContoller;
    private double StartAngle = 20;
    /** Creates a new ExampleSubsystem. */
    public ArmSubsystem() {
        //motor setup
        mShoulderMotor = new CANSparkMax(ArmConstants.kShoulderMotorId,MotorType.kBrushed);
        mWristMotor = new CANSparkMax(ArmConstants.kWristMotorId,MotorType.kBrushed);
        //TODO check if motors are brushed or brushless, if brushed check if Idle Mode Setting is needed
        mShoulderMotor.setIdleMode(IdleMode.kBrake); 
        mWristMotor.setIdleMode(IdleMode.kBrake);

        //PID setup
        mShoulderPIDContoller = initinalizePIDController(mShoulderMotor, ArmConstants.kShoulderPID);

        mWristPIDContoller = initinalizePIDController(mWristMotor, ArmConstants.kWristPID);
        //TODO make sure to add absolute encoders
        //should create a way to test where if the joystick button is pressed the angle the motors go to is incremented by ten
        
    }

    private SparkMaxPIDController initinalizePIDController(CANSparkMax mMotor, PIDConstants kPIDConstants) {
        SparkMaxPIDController targPidController = mMotor.getPIDController();
        mWristPIDContoller.setP(kPIDConstants.kP);
        mWristPIDContoller.setI(kPIDConstants.kI);
        mWristPIDContoller.setD(kPIDConstants.kD);
        mWristPIDContoller.setReference(StartAngle, CANSparkMax.ControlType.kPosition, 0);
        return targPidController;
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
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
