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

    private static ArmSubsystem mInstance;
    
    private ArmState mState;

    private final MotorAbsEncoderComboSubsystem mPivotMotor;
    private final MotorAbsEncoderComboSubsystem mWristMotor;
    // private final CANSparkMax mPivotFollower;

    // private final CANSparkMax mExtenderMotor;
    // private final SparkMaxPIDController mExtenderPIDController;

    // private final CANSparkMax mWristMotor;
    // private final SparkMaxPIDController mWristPIDController;

    public ArmSubsystem() {
        mState = ArmState.RETRACTED;
        System.out.println("arm created");
        mPivotMotor = new MotorAbsEncoderComboSubsystem(kPivotMotorConstants, true);  
        mWristMotor = new MotorAbsEncoderComboSubsystem(kWristMotorConstants, false);
        // mPivotFollower = kNeoConstructor.apply(kPivotFollowerID);
        // mPivotFollower.follow(mPivotMotor.getMotor()); //TODO check if this needs to be reversed

    //     mExtenderMotor = configureExtenderMotor(k775Constructor.apply(kExtenderMotorID));
    //     mExtenderPIDController = mExtenderMotor.getPIDController();
    //     mExtenderPIDController.setFeedbackDevice(mExtenderMotor.getAbsoluteEncoder(Type.kDutyCycle));

    //     mWristMotor = configureWristMotor(kNeoConstructor.apply(kWristMotorID));
    //     mWristPIDController = mWristMotor.getPIDController();
    //     mWristPIDController.setFeedbackDevice(mWristMotor.getAbsoluteEncoder(Type.kDutyCycle));
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

    // private double getArmRadius() {
    //     double rotations = (mExtenderMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition())/(2*Math.PI);
    //     return rotations * kNumOfArmSegments*(1/(kThreadsPerInch));
    // }
    // private void setArmRadius(double meters){
    //     double rotations = (meters)/(kNumOfArmSegments*(1/(kThreadsPerInch)));
    //     mExtenderPIDController.setReference(rotations, ControlType.kPosition);
    // }

    //TODO confirm that the +s and -s are correct for the conversions (as in whether or not they should be reversed)
    // private Rotation2d getLeveledWristAngle() {
    //     return new Rotation2d(mWristMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition() - mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
    // }
    // private void setLeveledWristAngle(Rotation2d rotation) {
    //    mWristPIDController.setReference(rotation.getRadians() + mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition(), ControlType.kPosition);
    // }

    public ArmPosition getPosition() {
        return new ArmPosition(
            0,
            new Rotation2d(mPivotMotor.getPosition()),
            new Rotation2d(mWristMotor.getPosition())
            // new Rotation2d(0)
        );
    }

    public ArmState getState() {
        return mState; //This will get the desired state
    }

    // //TODO determine offsets for absolute encoders
    private void setDesiredPosition(ArmPosition state) {
        mPivotMotor.setReference(state.armTheta);
        mWristMotor.setReference(state.wristTheta);
        mWristMotor.setReference(getCorrectAngle(state.armTheta));
        System.out.println("testing123123");
        // TODO add way for driver to interact
        // setLeveledWristAngle(state.wristTheta);
        // setArmRadius(state.armRadius);
    }
    private void setDesiredState(ArmState state) {
        setDesiredPosition(new ArmPosition(state.armRadius, state.armTheta, state.wristTheta));
    }

    private Rotation2d getCorrectAngle(Rotation2d armAngle){
        //assuming 0 on the arm is straight up
        //assuming 0 on the wrist is level with arm (makes a straight line), specifically 180 is level with the arm, 0 is opposite
        // arm ---- 0 ---- wrist 
        //the wrist angle will be the the arm angle - 90
        //ask val if this is unclear or not working
        return Rotation2d.fromDegrees(270).minus(armAngle);
    }

    public void setState(ArmState state) {
        mState = state;
        setDesiredState(mState);
    }

    public Rotation2d getRef(){
        return mPivotMotor.getCurrentReference();
    }

    //TODO make a way 
    public void transformState(double exstensionDelta, Rotation2d armDelta, Rotation2d wristDelta){
        ArmState current = getState();
        ArmPosition transformed = new ArmPosition(current.armRadius - exstensionDelta, current.armTheta.minus(armDelta), current.wristTheta.minus(wristDelta));

    }

    /**
     * Configures the pivot motor.
     * @param motor the pivot motor.
     * @return the pivot motor (for chaining).
     */
    public CANSparkMax configurePivotMotor(CANSparkMax motor) {
        motor.setIdleMode(IdleMode.kBrake);
        
        motor.getAbsoluteEncoder(Type.kDutyCycle).setPositionConversionFactor(kPivotPositionConversionFactor);
        System.out.println("okay setting up PID");
        motor.getPIDController().setP(kPivotP);
        motor.getPIDController().setI(kPivotI);
        motor.getPIDController().setD(kPivotD);
        System.out.println("PID values:\n"+motor.getPIDController().getP()+"\n"+motor.getPIDController().getI()+"\n"+motor.getPIDController().getD());
        motor.getPIDController().setPositionPIDWrappingMaxInput(Math.PI*2);
        motor.getPIDController().setPositionPIDWrappingMinInput(0);
        motor.getPIDController().setPositionPIDWrappingEnabled(true);
        

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

        motor.getPIDController().setP(kWristP);
        motor.getPIDController().setI(kWristI);
        motor.getPIDController().setD(kWristD);
        motor.getPIDController().setPositionPIDWrappingEnabled(true);

        return motor;
    }

    // @Override
    // public void periodic() {
    //     System.out.println("testing testing testing");
    //     setDesiredState(mState);
    //     System.out.println(mState.armTheta);
    // }
}
