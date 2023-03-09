package com.spartronics4915.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
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

            Rotation2d pivotArmAngle = mPivot.getHorizonPosition();
            Rotation2d wristArmAngle = mWrist.getHorizonPosition();

            Rotation2d wristAngleWithEarth = pivotArmAngle.plus(wristArmAngle);
            return  wristAngleWithEarth;
        }
    }
    
    /**
     * Represents the preset values for the arm
     */
    public enum ArmState {
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
        CUBE_LEVEL_2(kCubeLevel2Constants);

        // CONE_LEVEL_3(kConeLevel3Constants);

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

    private ArmState mDesiredState;

    private final PivotComponent mPivotMotor;
    private final WristComponent mWristMotor;
    private final CANSparkMax mPivotFollower;

    private final ExtenderComponent mExtenderComponent;
    private final Intake mIntake;

    public ArmSubsystem() {
        mDesiredState = ArmState.RETRACTED;
        System.out.println("arm created");
        mPivotMotor = new PivotComponent();
        mWristMotor = new WristComponent(mPivotMotor);
        mPivotFollower = kNeoConstructor.apply(kPivotFollowerID);
        mPivotFollower.restoreFactoryDefaults();
        mPivotFollower.follow(mPivotMotor.getMotor(), true);
        mPivotFollower.setSmartCurrentLimit(20);
        mPivotFollower.setIdleMode(IdleMode.kBrake);
        mExtenderComponent = new ExtenderComponent();

        mIntake = Intake.getInstance();
        
        // if(mWristMotor != null) {
        //     mWristMotor.setAngleWithEarthProvider(new WristAngleProvider(mPivotMotor, mWristMotor));
        // }
        // mPivotMotor.setActive(true);
    }

    public ExtenderComponent getExtender() {
        return mExtenderComponent;
    }
    
    public ArmJointAbstractComponent[] getMotors() {
        ArmJointAbstractComponent[] x = { mPivotMotor, mWristMotor };
        return x;
    }

    public PivotComponent getPivot() {
        return mPivotMotor;
    }

    public WristComponent getWrist() {
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
                mExtenderComponent.getPosition(),
                mPivotMotor.getHorizonPosition(),
                new Rotation2d(0)
        );

        }
        return new ArmPosition(
                mExtenderComponent.getPosition(),
                mPivotMotor.getHorizonPosition(),
                mWristMotor.getHorizonPosition()
        // new Rotation2d(0)
        );
    }

    public ArmPosition  getLocalReference() {
        Rotation2d pivotReference = mPivotMotor.nativeToHorizon(mPivotMotor.getCurrentReference());
        Rotation2d wristReference = mWristMotor.nativeToHorizon(mWristMotor.getCurrentReference());
        double extenderReference = mExtenderComponent.getReference();

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
        Rotation2d newWristAngle = (newPivotAngle.getDegrees() > 90) ? Rotation2d.fromDegrees(180).plus(localArmPosition.wristTheta.unaryMinus()) : localArmPosition.wristTheta;
        newWristAngle = newWristAngle.plus(localArmPosition.armTheta);

        var newExtension = localArmPosition.armRadius;

        return new ArmPosition(newExtension, newPivotAngle, newWristAngle);
    }

    public ArmPosition globalToLocalState(ArmPosition globalArmPosition) {
        // localArmState are the actual angles that wrists are at.
        // Global state has all of the angles in global coordinates (0 is the horizon)

        Rotation2d newPivotAngle = globalArmPosition.armTheta;
        Rotation2d newWristAngle = globalArmPosition.wristTheta.minus(globalArmPosition.armTheta);
        newWristAngle = (newPivotAngle.getDegrees() > 90) ? Rotation2d.fromDegrees(180).minus(newWristAngle).unaryMinus() : newWristAngle;
        var newExtension = globalArmPosition.armRadius;

        return new ArmPosition(newExtension, newPivotAngle, newWristAngle);
    }

    // TODO determine zero offsets
    // TODO add extender motor
    private void setDesiredLocalPosition(ArmPosition state) {
        // mExtenderComponent.extendToNInches(state.armRadius).schedule(); 
        mPivotMotor.setHorizonReference(state.armTheta);
        mWristMotor.setHorizonReference(state.wristTheta);
    }

    private void setDesiredGlobalPosition(ArmPosition pos) {
        setDesiredLocalPosition(globalToLocalState(pos));
    }

    public void setDesiredLocalState(ArmState state) {
        // Global state has all of the angles in global coordinates (0 is the horizon)
        mDesiredState = state;
        setDesiredLocalPosition(new ArmPosition(state.armRadius, state.armTheta, state.wristTheta));
    }

    public void setDesiredGlobalState(ArmState state) {
        // Global state has all of the angles in global coordinates (0 is the horizon)
        mDesiredState = state;
        setDesiredGlobalPosition(new ArmPosition(state.armRadius, state.armTheta, state.wristTheta));
    }

    private Rotation2d getCorrectLocalWristAngle(Rotation2d armAngle, Rotation2d wristAngle) {
        // assuming 0 on the arm is straight up
        // assuming 0 on the wrist is level with arm (makes a straight line),
        // specifically 180 is level with the arm, 0 is opposite
        // arm ---- 0 ---- wrist
        // the wrist angle will be the the arm angle - 90
        // ask val if this is unclear or not working
        return armAngle.unaryMinus();
    }


    public Rotation2d getRef() {
        return mPivotMotor.getCurrentReference();
    }

    // TODO make a way
    /**
     * this method transform the arm based on its current position
     * @param exstensionDelta distance in inches to tranform the arm by
     * @param armDelta  amount to rotate the arm by
     * @param wristDelta amount to rotate the wrist by
     */
    public void transformPosition(double exstensionDelta, Rotation2d armDelta, Rotation2d wristDelta) {
        ArmPosition current = getLocalReference();
        ArmPosition transformed = new ArmPosition(
            current.armRadius + exstensionDelta, 
            current.armTheta.plus(armDelta),
            current.wristTheta.plus(wristDelta));
        setDesiredLocalPosition(transformed);
        System.out.println("Transform Called " + " " + transformed.wristTheta.getDegrees() + " " + transformed.armTheta.getDegrees() + " " + wristDelta);

    }
    

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        mPivotMotor.onPeriodic();
        mWristMotor.onPeriodic();
        mExtenderComponent.onPeriodic();
    }
    // public void transformState(Rotation2d armDelta, Rotation2d wristDelta) {
    //     ArmPosition current = getPosition();
        
    //     mPivotMotor.setReference(current.armTheta.minus(armDelta));

    //     if (mWristMotor != null) {
    //         mWristMotor.setReference(getCorrectAngle(current.armTheta).minus(wristDelta));
    //     }
    // }
}