package com.spartronics4915.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.spartronics4915.frc2023.Constants.Arm.ArmPositionConstants.ArmSettingsConstants;
import com.spartronics4915.frc2023.commands.ArmCommands;
import com.spartronics4915.frc2023.subsystems.MotorAbsEncoderComboSubsystem.AngleWithEarthProvider;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2023.Constants.Arm.*;

/**
 * this is the main subsytem responsible for controlling and managing the arm
 */
public class ArmSubsystem extends SubsystemBase {

    private class WristAngleProvider implements AngleWithEarthProvider {
        MotorAbsEncoderComboSubsystem mPivot, mWrist;
        public WristAngleProvider(MotorAbsEncoderComboSubsystem pivot, MotorAbsEncoderComboSubsystem wrist)
        {
            mPivot = pivot;
            mWrist = wrist;
        }

        public Rotation2d getAngleWithEarth() {

            Rotation2d pivotArmAngle = mPivot.getArmPosition();
            Rotation2d wristArmAngle = mWrist.getArmPosition();

            Rotation2d wristAngleWithEarth = pivotArmAngle.plus(wristArmAngle);
            return  wristAngleWithEarth;
        }
    }
    
    /**
     * Represents the preset values for the arm
     */
    public enum ArmState {
        TUCK_INTERMEDIATE(kReadyForTuck),
        RETRACTED(kRetractedConstants),
        // GRAB_UPRIGHT(kGrabUprightConstants),
        // GRAB_FALLEN(kArmLowConstants),
        // ARM_LOW(kArmLowConstants),
        // ARM_HIGH(kArmHighConstants),
        // ARM_LEVEL(kArmLevelConstants),
        RETRACTED_PRIOR(kPriorRetracted),
        FLOOR_POS(kFloorPositionConstants),
        DOUBLE_SUBSTATION(kDoubleSubstationConstants),
        CONE_LEVEL_1(kConeLevel1Constants),
        CONE_LEVEL_2(kConeLevel2Constants),
        CUBE_LEVEL_1(kCubeLevel1Constants),
        CUBE_LEVEL_2(kCubeLevel2Constants),

        SHOOT_HIGH_CUBE(kCubeTopShootConstants);

        // CONE_LEVEL_3(kConeLevel3Constants);

        public final double armRadius;
        public final Rotation2d armTheta;
        public final Rotation2d wristTheta;
        public final double outSpeed;

        private ArmState(ArmSettingsConstants armConstants) {
            var armPositionConstants = armConstants.mArmPositionConstants;
            armRadius = armPositionConstants.armRadius;
            armTheta = armPositionConstants.armTheta;
            wristTheta = armPositionConstants.wristTheta;
            outSpeed = armConstants.mOutSpeed;
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

    private ArmState mDesiredState;

    private final MotorAbsEncoderComboSubsystem mPivotMotor;
    private final MotorAbsEncoderComboSubsystem mWristMotor;
    private final CANSparkMax mPivotFollower;

    private final BeltExtenderSubsystem mExtenderSubsystem;
    private final Intake mIntake;

    public ArmSubsystem() {
        mDesiredState = ArmState.RETRACTED;
        System.out.println("arm created");
        mPivotMotor = new MotorAbsEncoderComboSubsystem(kPivotMotorConstants, MotorType.kBrushless);
        mWristMotor = new MotorAbsEncoderComboSubsystem(kWristMotorConstants, MotorType.kBrushed);
        mPivotFollower = kNeoConstructor.apply(kPivotFollowerID);
        mPivotFollower.restoreFactoryDefaults();
        mPivotFollower.follow(mPivotMotor.getMotor(), true);
        mPivotFollower.setSmartCurrentLimit(60);
        mPivotFollower.setIdleMode(IdleMode.kBrake);
        mExtenderSubsystem = new BeltExtenderSubsystem(mPivotMotor);

        mIntake = Intake.getInstance();
        
        if(mWristMotor != null) {
            mWristMotor.setAngleWithEarthProvider(new WristAngleProvider(mPivotMotor, mWristMotor));
        }
        
        
        // mPivotMotor.setActive(false);
    }

    public void makeModeledPositionsMatchPhysical() {

    }

    public Intake getIntake() {
        return mIntake;
    }
    
    public BeltExtenderSubsystem getExtender() {
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

    public ArmPosition getLocalPosition() {
        
        if(mWristMotor == null) {
            return new ArmPosition(
                mExtenderSubsystem.getPosition(),
                mPivotMotor.getArmPosition(),
                new Rotation2d(0)
        );

        }
        return new ArmPosition(
                mExtenderSubsystem.getPosition(),
                mPivotMotor.getArmPosition(),
                mWristMotor.getArmPosition()
        // new Rotation2d(0)
        );
    }

    public ArmPosition getGlobalPosition () {
        return localToGlobalPosition(this.getLocalPosition());
    }

    public ArmPosition  getLocalReference() {
        Rotation2d pivotReference = mPivotMotor.getCurrentReferenceArm();
        Rotation2d wristReference = mWristMotor.getCurrentReferenceArm();
        double extenderReference = mExtenderSubsystem.getTarget();

        return new ArmPosition(extenderReference, pivotReference, wristReference);

    }

    public ArmState getDesiredGlobalState() {
        // Global state has all of the angles in global coordinates (0 is the horizon)
        return mDesiredState; // This will get the desired state
    }

    public ArmPosition localToGlobalPosition(ArmPosition localArmPosition) {
        // localArmState are the actual angles that wrists are at.
        // Global state has all of the angles in global coordinates (0 is the horizon)

        Rotation2d newPivotAngle = localArmPosition.armTheta;
        Rotation2d newWristAngle = localArmPosition.wristTheta.plus(localArmPosition.armTheta);
        var newExtension = localArmPosition.armRadius;

        return new ArmPosition(newExtension, newPivotAngle, newWristAngle);
    }

    public ArmPosition globalToLocalState(ArmPosition globalArmPosition) {
        // localArmState are the actual angles that wrists are at.
        // Global state has all of the angles in global coordinates (0 is the horizon)

        Rotation2d newPivotAngle = globalArmPosition.armTheta;
        Rotation2d newWristAngle = globalArmPosition.wristTheta.minus(globalArmPosition.armTheta);
        var newExtension = globalArmPosition.armRadius;

        return new ArmPosition(newExtension, newPivotAngle, newWristAngle);
    }
    

    // TODO determine zero offsets
    // TODO add extender motor
    private void setDesiredLocalPivotWristPosition(Rotation2d armTheta, Rotation2d wristTheta) {
        mPivotMotor.setArmReference(armTheta);
        mWristMotor.setArmReference(wristTheta);
    }

    // public CommandBase setDesiredLocalStateCommand(ArmPosition state) {
    //     return runOnce(() -> setDesiredLocalPosition(state));
    // }

    // private void setDesiredGlobalPosition(ArmPosition pos) {
    //     setDesiredLocalPosition(globalToLocalState(pos));
    // }

    public void setDesiredLocalState(ArmState state) {
        // Global state has all of the angles in global coordinates (0 is the horizon)
        mDesiredState = state;
        setDesiredLocalPivotWristPosition(state.armTheta, state.wristTheta);
    }

    // public void setDesiredGlobalState(ArmState state) {
    //     // Global state has all of the angles in global coordinates (0 is the horizon)
    //     mDesiredState = state;
    //     setDesiredGlobalPosition(new ArmPosition(state.armRadius, state.armTheta, state.wristTheta));
    // }

    private Rotation2d getCorrectAngleDEPRECATE(Rotation2d armAngle) {
        // assuming 0 on the arm is straight up
        // assuming 0 on the wrist is level with arm (makes a straight line),
        // specifically 180 is level with the arm, 0 is opposite
        // arm ---- 0 ---- wrist
        // the wrist angle will be the the arm angle - 90
        // ask val if this is unclear or not working
        return armAngle.unaryMinus();
    }


    public void clearReference() {
        mPivotMotor.clearReference();
        mWristMotor.clearReference();
        mExtenderSubsystem.stopExtenderAndMatchTargetToPhysical();
    }
    
    public void stopPivot() {
        mPivotMotor.stopMotor();
    }

    // // Probably should remove
    // public Rotation2d getRef() {
    //     return mPivotMotor.getCurrentReference();
    // }

    // TODO make a way
    /**
     * this method transform the arm based on its current position
     * @param exstensionDelta distance in inches to tranform the arm by - Extension currently not supported.
     * @param armDelta  amount to rotate the arm by
     * @param wristDelta amount to rotate the wrist by
     */
    public void transformPosition(double exstensionDelta, Rotation2d armDelta, Rotation2d wristDelta) {
        ArmPosition current = getLocalReference();
        ArmPosition transformed = new ArmPosition(
            current.armRadius + exstensionDelta, 
            current.armTheta.plus(armDelta),
            current.wristTheta.plus(wristDelta));
        setDesiredLocalPivotWristPosition(transformed.armTheta, transformed.wristTheta);
        System.out.println("Transform Called " + " " + transformed.wristTheta.getDegrees() + " " + transformed.wristTheta.getRadians() + " " + wristDelta);

    }
    
    // public void transformState(Rotation2d armDelta, Rotation2d wristDelta) {
    //     ArmPosition current = getPosition();
        
    //     mPivotMotor.setReference(current.armTheta.minus(armDelta));

    //     if (mWristMotor != null) {
    //         mWristMotor.setReference(getCorrectAngle(current.armTheta).minus(wristDelta));
    //     }
    // }
}