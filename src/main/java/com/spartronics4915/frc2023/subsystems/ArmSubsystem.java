package com.spartronics4915.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2023.Constants.Arm.*;

public class ArmSubsystem extends SubsystemBase {
    /**
     * Represents the position of the arm and wrist.
     */
    public enum ArmState {
        RETRACTED(kRetractedConstants),
        GRAB_UPRIGHT(kGrabUprightConstants),
        GRAB_FALLEN(kArmLowConstants),
        ARM_LOW(kArmLowConstants),
        ARM_HIGH(kArmHighConstants),
        ARM_LEVEL(kArmLevelConstants),
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

    // /**
    // * Represents the state of the arm. Contains the arm's position and the
    // intake's state.
    // */
    // public static final class ArmState { // I wish java 11 had records :(
    // public final ArmPosition armPosition;
    // public final IntakeState intakeState;

    // public ArmState(ArmPosition armPosition, IntakeState intakeState) {
    // this.armPosition = armPosition;
    // this.intakeState = intakeState;
    // }
    // }

    private static ArmSubsystem mInstance;

    private ArmState mState;

    private final MotorAbsEncoderComboSubsystem mPivotMotor;
    private final MotorAbsEncoderComboSubsystem mWristMotor;
    private final CANSparkMax mPivotFollower;

    private final ExtenderSubsystem mExtenderSubsystem;
    // private final SparkMaxPIDController mExtenderPIDController;

    // private final CANSparkMax mWristMotor;
    // private final SparkMaxPIDController mWristPIDController;

    public ArmSubsystem() {
        mState = ArmState.RETRACTED;
        System.out.println("arm created");
        mPivotMotor = new MotorAbsEncoderComboSubsystem(kPivotMotorConstants, true);
        mWristMotor = new MotorAbsEncoderComboSubsystem(kWristMotorConstants, true);
        // mWristMotor = new MotorAbsEncoderComboSubsystem(kWristMotorConstants, false);
        mPivotFollower = kNeoConstructor.apply(kPivotFollowerID);
        mPivotFollower.restoreFactoryDefaults();
        mPivotFollower.follow(mPivotMotor.getMotor(), true);
        mPivotFollower.setSmartCurrentLimit(10);

        mExtenderSubsystem = new ExtenderSubsystem(17);

    }

    public ExtenderSubsystem getExtender() {
        return mExtenderSubsystem;
    }
    
    public MotorAbsEncoderComboSubsystem[] getMotors() {
        MotorAbsEncoderComboSubsystem[] x = { mPivotMotor, mWristMotor };
        return x;
    }

    public MotorAbsEncoderComboSubsystem getPivot() {
        return mPivotMotor;
    }

    public MotorAbsEncoderComboSubsystem getWrist() {
        return mWristMotor;
    }


    public static ArmSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ArmSubsystem();
        }
        return mInstance;
    }

    public ArmPosition getPosition() {
        return new ArmPosition(
                0,
                mPivotMotor.getArmPosition(),
                mWristMotor.getArmPosition()
        // new Rotation2d(0)
        );
    }

    public ArmState getState() {
        return mState; // This will get the desired state
    }

    // TODO determine zero offsets
    // TODO add extender motor
    private void setDesiredPosition(ArmPosition state) {
        mPivotMotor.setReference(state.armTheta);

        if (mWristMotor != null) {
            // mWristMotor.setReference(state.wristTheta);
            mWristMotor.setReference(getCorrectAngle(state.armTheta));
        }
    }

    private void setDesiredState(ArmState state) {
        setDesiredPosition(new ArmPosition(state.armRadius, state.armTheta, state.wristTheta));
    }

    private Rotation2d getCorrectAngle(Rotation2d armAngle) {
        // assuming 0 on the arm is straight up
        // assuming 0 on the wrist is level with arm (makes a straight line),
        // specifically 180 is level with the arm, 0 is opposite
        // arm ---- 0 ---- wrist
        // the wrist angle will be the the arm angle - 90
        // ask val if this is unclear or not working
        return armAngle.unaryMinus();
    }

    public void setState(ArmState state) {
        mState = state;
        setDesiredState(mState);
    }

    public Rotation2d getRef() {
        return mPivotMotor.getCurrentReference();
    }

    // TODO make a way
    public void transformState(double exstensionDelta, Rotation2d armDelta, Rotation2d wristDelta) {
        ArmState current = getState();
        ArmPosition transformed = new ArmPosition(current.armRadius - exstensionDelta, current.armTheta.minus(armDelta),
                current.wristTheta.minus(wristDelta));
        setDesiredPosition(transformed);
    }
}