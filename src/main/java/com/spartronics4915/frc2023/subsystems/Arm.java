package com.spartronics4915.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2023.Constants.Arm.*;

public class Arm extends SubsystemBase {
    /**
     * Represents the position of the arm and wrist.
     */
    public enum ArmState {
        RETRACTED(kRetractedConstants),
        GRAB_UPRIGHT(kGrabUprightConstants),
        GRAB_FALLEN(kGrabFallenConstants),
        LEVEL_1(kLevel1Constants),
        LEVEL_2(kLevel2Constants),
        LEVEL_3(kLevel3Constants);

        public final double armRadius;
        public final Rotation2d armTheta;
        public final Rotation2d wristTheta;

        private ArmState(ArmPositionConstants armPositionConstants) {
            armRadius = armPositionConstants.armRadius;
            armTheta = armPositionConstants.armTheta;
            wristTheta = armPositionConstants.wristTheta;
        }
    }

    public static class ArmPosition {
        public final double armRadius;
        public final Rotation2d armTheta;
        public final Rotation2d wristTheta;

        public ArmPosition(double armRadius, Rotation2d armTheta, Rotation2d wristTheta) {
            this.armRadius = armRadius;
            this.armTheta = armTheta;
            this.wristTheta = wristTheta;
        }
    }

    private static Arm mInstance;

    private ArmState mState;

    private final CANSparkMax mPivotMotor;
    private final SparkMaxPIDController mPivotPIDController;
    private final ArmFeedforward mPivotFeedforward;

    private final CANSparkMax mPivotFollower;

    private final CANSparkMax mExtenderMotor;
    private final SparkMaxPIDController mExtenderPIDController;

    private final CANSparkMax mWristMotor;
    private final SparkMaxPIDController mWristPIDController;

    private Arm() {
        mState = ArmState.RETRACTED;

        mPivotMotor = configurePivotMotor(kNeoConstructor.apply(kPivotMotorID));
        mPivotPIDController = mPivotMotor.getPIDController();
        mPivotFollower = kNeoConstructor.apply(kPivotFollowerID);
        mPivotFollower.follow(mPivotMotor); // TODO check if this needs to be reversed

        mPivotFeedforward = new ArmFeedforward(kPivotS, kPivotG, kPivotV, kPivotA);

        mExtenderMotor = configureExtenderMotor(k775Constructor.apply(kExtenderMotorID));
        mExtenderPIDController = mExtenderMotor.getPIDController();

        mWristMotor = configureWristMotor(kNeoConstructor.apply(kWristMotorID));
        mWristPIDController = mWristMotor.getPIDController();
    }

    public static Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm();
        }
        return mInstance;
    }

    private double getArmRadius() {
        double rotations = (mExtenderMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition()) / (2 * Math.PI);
        return rotations * kNumOfArmSegments * (1 / (kThreadsPerInch));
    }

    private void setArmRadius(double meters) {
        double rotations = (meters) / (kNumOfArmSegments * (1 / (kThreadsPerInch)));
        mExtenderPIDController.setReference(rotations, ControlType.kPosition);
    }

    // TODO confirm that the +s and -s are correct for the conversions
    private Rotation2d getLeveledWristAngle() {
        return new Rotation2d(mWristMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition()
                - mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
    }

    private void setLeveledWristAngle(Rotation2d rotation) {
        mWristPIDController.setReference(
                rotation.getRadians() + mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition(),
                ControlType.kPosition);
    }

    public ArmPosition getPosition() {
        return new ArmPosition(
                mExtenderMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition(),
                new Rotation2d(mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition()),
                new Rotation2d(mWristMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition()));
    }

    public ArmState getState() {
        return mState; // This will get the desired state
    }

    // TODO determine offsets for absolute encoders
    private void setDesiredState(ArmState state) {
        mPivotPIDController.setReference(
                state.armTheta.getRadians(),
                ControlType.kPosition,
                0,
                mPivotFeedforward.calculate(
                        getState().armTheta.getRadians(),
                        mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle).getVelocity()),
                ArbFFUnits.kVoltage);
        mExtenderPIDController.setReference(state.armRadius, ControlType.kPosition);
        mWristPIDController.setReference(state.wristTheta.minus(getState().armTheta).getRadians(),
                ControlType.kPosition);
    }

    public void setState(ArmState state) {
        mState = state;
    }

    /**
     * Configures the pivot motor.
     * 
     * @param motor the pivot motor.
     * @return the pivot motor (for chaining).
     */
    public CANSparkMax configurePivotMotor(CANSparkMax motor) {
        motor.setIdleMode(IdleMode.kBrake);

        motor.getAbsoluteEncoder(Type.kDutyCycle).setPositionConversionFactor(kPivotPositionConversionFactor);
        motor.getAbsoluteEncoder(Type.kDutyCycle).setVelocityConversionFactor(kPivotVelocityConversionFactor);

        motor.getPIDController().setFeedbackDevice(mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle));

        motor.getPIDController().setP(kPivotP);
        motor.getPIDController().setI(kPivotI);
        motor.getPIDController().setD(kPivotD);
        motor.getPIDController().setFF(kPivotFF);

        return motor;
    }

    public CANSparkMax configureExtenderMotor(CANSparkMax motor) {
        motor.setIdleMode(IdleMode.kBrake);

        motor.getAbsoluteEncoder(Type.kDutyCycle).setPositionConversionFactor(kExtenderPositionConversionFactor);

        motor.getPIDController().setP(kExtenderP);
        motor.getPIDController().setI(kExtenderI);
        motor.getPIDController().setD(kExtenderD);
        motor.getPIDController().setPositionPIDWrappingEnabled(false);

        return motor;
    }

    public CANSparkMax configureWristMotor(CANSparkMax motor) {
        motor.setIdleMode(IdleMode.kBrake);

        motor.getAbsoluteEncoder(Type.kDutyCycle).setPositionConversionFactor(kWristPositionConversionFactor);

        motor.getPIDController().setFeedbackDevice(mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle));

        motor.getPIDController().setP(kWristP);
        motor.getPIDController().setI(kWristI);
        motor.getPIDController().setD(kWristD);
        motor.getPIDController().setFF(kWristFF);

        return motor;
    }

    @Override
    public void periodic() {
        setDesiredState(mState);
    }
}
