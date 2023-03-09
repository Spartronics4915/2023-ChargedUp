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
        SHOOT_CUBE,
        PLACE_CUBE,
        PLACE_CONE,
        OUT
    }

    private static Intake mInstance;

    private IntakeState mState;

    private final CANSparkMax mMotor;

    private Intake() {
        mMotor = kMotorConstructor.apply(kIntakeMotorID);
        mMotor.restoreFactoryDefaults();
        mMotor.setInverted(kIsInverted);
        mMotor.setIdleMode(IdleMode.kCoast);
        mMotor.setSmartCurrentLimit(40);
        mMotor.burnFlash();
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
            case IN: mMotor.set(kInSpeed); break;
            case SHOOT_CUBE: mMotor.set(kShootCubeSpeed); break;
            case PLACE_CUBE: mMotor.set(kPlaceCubeSpeed); break;
            case PLACE_CONE: mMotor.set(kPlaceConeSpeed); break;
            case OUT: mMotor.set(kDefaultOutSpeed); break; 
            default: mMotor.set(kOffSpeed);
        }
    }
}
