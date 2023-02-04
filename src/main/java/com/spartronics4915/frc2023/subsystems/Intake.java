package com.spartronics4915.frc2023.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2023.Constants.Intake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

public class Intake extends SubsystemBase {
    /**
     * Represents the state of the intake.
     */
    public enum IntakeState {
        IN,
        OFF,
        OUT
    }

    private static Intake mInstance;

    private IntakeState mState;

    private final CANSparkMax mMotor;

    private Intake() {
        mMotor = kMotorConstructor.apply(kIntakeMotorID);
        mMotor.setInverted(kIsInverted);
        mMotor.setIdleMode(IdleMode.kBrake);

        mState = IntakeState.OFF;
    }

    public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    public void setState(IntakeState state) {
        mState = state;
    }

    public IntakeState getState() {
        return mState;
    }


    @Override
    public void periodic() {
        switch(mState) {
            case IN: mMotor.set(kInSpeed);
            case OUT: mMotor.set(kOutSpeed);
            default: mMotor.set(0);
        }
    }
}
