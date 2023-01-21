// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.spartronics4915.frc2023.Constants.ArmConstants.ClawConstants;
import com.spartronics4915.frc2023.Constants.ArmConstants.PIDConstants;

public class ClawSubsystem extends SubsystemBase {
    private CANSparkMax mClawMotor;
    private SparkMaxPIDController mClawPID;
    private DigitalInput mLimitSwitch;
    /** Creates a new ExampleSubsystem. */
    public ClawSubsystem() {
        mClawMotor = new CANSparkMax(ClawConstants.kClawMotorID, MotorType.kBrushless);
        
        mLimitSwitch = new DigitalInput(ClawConstants.klimitSwitchID);
        mClawPID = mClawMotor.getPIDController();
        mClawPID.setP(ClawConstants.kClawMotorPID.kP);
        mClawPID.setI(ClawConstants.kClawMotorPID.kI);
        mClawPID.setD(ClawConstants.kClawMotorPID.kD);
        //TODO setup the encoders for this PID and maybe make the on-off transition more smooth
    }
   

    public void motorIn() {
        mClawPID.setReference(ClawConstants.kInSpeed, ControlType.kVelocity);
    }
    public void motorOut() {
        mClawPID.setReference(-ClawConstants.kOutSpeed, ControlType.kVelocity);
    }
    public void motorOff() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.        mClawMotor.set(mDefaultSpeed);
        mClawPID.setReference(0, ControlType.kVelocity);
    }

    public boolean limitSwitch() {
        // Query some boolean state, such as a digital sensor.
        return mLimitSwitch.get();
    }

    @Override
    public void periodic() {
        // This method will /be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
