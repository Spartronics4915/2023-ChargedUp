package com.spartronics4915.frc2023.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
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

    double outSpeed;
    private static Intake mInstance;

    private IntakeState mState;

    private final CANSparkMax mMotor;

    private Intake() {
        mMotor = kMotorConstructor.apply(kIntakeMotorID);
        mMotor.restoreFactoryDefaults();
        mMotor.setInverted(kIsInverted);
        mMotor.setIdleMode(IdleMode.kBrake);
        mMotor.setSmartCurrentLimit(40);
        mMotor.burnFlash();
        mState = IntakeState.OFF;
        outSpeed = kDefaultOutSpeed;
    }

    public double getOutSpeed() {
        return outSpeed;
    }

    public void setOutSpeed(double newSpeed) {
        outSpeed = newSpeed;

    }

    public CommandBase setOutSpeedCommand(double newSpeed) {

        return this.runOnce(() -> setOutSpeed(
                newSpeed));
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
        switch (mState) {
            case IN:
                mMotor.set(kInSpeed);
                break;
            case OUT:
                mMotor.set(outSpeed);
                System.out.println("OutSpeed: " + outSpeed);
                break;
            default:
                mMotor.set(-0.02);
        }
    }
}
