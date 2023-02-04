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

    // /**
    //  * Represents the state of the arm. Contains the arm's position and the intake's state. 
    //  */
    // public static final class ArmState { // I wish java 11 had records :(
    //     public final ArmPosition armPosition;
    //     public final IntakeState intakeState;

    //     public ArmState(ArmPosition armPosition, IntakeState intakeState) {
    //         this.armPosition = armPosition;
    //         this.intakeState = intakeState;
    //     }
    // }

    private static Arm mInstance;
    
    private ArmState mState;

    private final CANSparkMax mPivotMotor;
    private final SparkMaxPIDController mPivotPIDController;

    private final CANSparkMax mPivotFollower;

    private final CANSparkMax mExtenderMotor;
    private final SparkMaxPIDController mExtenderPIDController;

    private final CANSparkMax mWristMotor;
    private final SparkMaxPIDController mWristPIDController;

    private Arm() { // TODO: config motors
        mState = ArmState.RETRACTED;
        
        mPivotMotor = configurePivotMotor(kNeoConstructor.apply(kPivotMotorID));
        mPivotPIDController = mPivotMotor.getPIDController();
        mPivotPIDController.setFeedbackDevice(mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle));

        mPivotFollower = kNeoConstructor.apply(kPivotFollowerID);
        mPivotFollower.follow(mPivotMotor);

        mExtenderMotor = configureExtenderMotor(k775Constructor.apply(kExtenderMotorID));
        mExtenderPIDController = mExtenderMotor.getPIDController();
        mExtenderPIDController.setFeedbackDevice(mExtenderMotor.getAbsoluteEncoder(Type.kDutyCycle));

        mWristMotor = configureWristMotor(kNeoConstructor.apply(kWristMotorID));
        mWristPIDController = mWristMotor.getPIDController();
        mWristPIDController.setFeedbackDevice(mWristMotor.getAbsoluteEncoder(Type.kDutyCycle));
    }

    public static Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm();
        }
        return mInstance;
    }

    private double getArmRadius() {
        double rotations = (mExtenderMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition())/(2*Math.PI);
        return rotations * kNumOfArmSegments*(1/(kThreadsPerInch));
    }
    private void setArmRadius(double meters){
        double rotations = (meters)/(kNumOfArmSegments*(1/(kThreadsPerInch)));
        mExtenderPIDController.setReference(rotations, ControlType.kPosition);
    }

    //TODO make wrist's angle based on ground angle
    public ArmPosition getPosition() {
        return new ArmPosition(
            getArmRadius(),
            new Rotation2d(mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition()),
            new Rotation2d(mWristMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition())
        );
    }

    public ArmState getState() {
        return mState; //This will get the desired state
    }

    //FIXME need to convert distance to rotations, there are no conversions currently
    //TODO check if encoders need to be offset, (in rotation2d 0 is level with ground)
    //TODO add calculations to make 0 on wrist level with the ground (same with the arm pivot)
    private void setDesiredState(ArmState state) {
        mPivotPIDController.setReference(state.armTheta.getRadians(), ControlType.kPosition);
        mWristPIDController.setReference(state.wristTheta.getRadians(), ControlType.kPosition);
        setArmRadius(state.armRadius);
    }

    public void setState(ArmState state) {
        mState = state;
    }

    /**
     * Configures the pivot motor.
     * @param motor the pivot motor.
     * @return the pivot motor (for chaining).
     */
    public CANSparkMax configurePivotMotor(CANSparkMax motor) {
        motor.setIdleMode(IdleMode.kBrake);
        
        motor.getAbsoluteEncoder(Type.kDutyCycle).setPositionConversionFactor(kPivotPositionConversionFactor);
        
        motor.getPIDController().setP(kPivotP);
        motor.getPIDController().setI(kPivotI);
        motor.getPIDController().setD(kPivotD);
        
        return motor;
    }

    public CANSparkMax configureExtenderMotor(CANSparkMax motor) {
        motor.setIdleMode(IdleMode.kBrake);
        
        motor.getAbsoluteEncoder(Type.kDutyCycle).setPositionConversionFactor(kExtenderPositionConversionFactor);

        motor.getPIDController().setP(kExtenderP);
        motor.getPIDController().setI(kExtenderI);
        motor.getPIDController().setD(kExtenderD);
        
        return motor;
    }

    public CANSparkMax configureWristMotor(CANSparkMax motor) {
        motor.setIdleMode(IdleMode.kBrake);
        
        motor.getAbsoluteEncoder(Type.kDutyCycle).setPositionConversionFactor(kWristPositionConversionFactor);

        motor.getPIDController().setP(kWristP);
        motor.getPIDController().setI(kWristI);
        motor.getPIDController().setD(kWristD);

        return motor;
    }

    @Override
    public void periodic() {
        setDesiredState(mState);
    }
}