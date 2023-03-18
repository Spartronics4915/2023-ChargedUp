package com.spartronics4915.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.spartronics4915.frc2023.Constants.Arm;
import com.spartronics4915.frc2023.Constants.Arm.ArmMotorConstants;
import com.spartronics4915.frc2023.Constants.Arm.CanSparkMaxMotorConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ArmJointAbstractComponent {

    protected CANSparkMax mMotor;
    protected SparkMaxAbsoluteEncoder mAbsEncoder;
    protected SparkMaxPIDController mPIDController;
    protected final ArmMotorConstants kConstants;
    protected Rotation2d currentRefrence;
    protected boolean setRef;
    protected boolean mDisableMotor;
    protected double mModeledVelocity = 0;
    protected double mModeledPosition = 0;

    public ArmJointAbstractComponent(ArmMotorConstants constants) {
        super();
        kConstants = constants;
        motorInit();
        encoderInit();
        pidControllerInit();
        setRef = false;
        currentRefrence = HorizonToNative(Rotation2d.fromDegrees(0));
        
    }

    protected void motorInit() {
        CanSparkMaxMotorConstants motorConstants =  kConstants.kMotorConstants;
        mMotor = new CANSparkMax(motorConstants.kMotorID, motorConstants.kMotorType);
        mMotor.setIdleMode(motorConstants.kIdleMode);
        mMotor.setInverted(motorConstants.kInverted);

        mMotor.setSmartCurrentLimit(20);
    }
    
    protected void encoderInit() {
        CanSparkMaxMotorConstants motorConstants =  kConstants.kMotorConstants;
        mAbsEncoder = mMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        
        // mAbsEncoder.setZeroOffset(0.5);
        // //if zero offset wasn't actually applied then the encoder is unplugged
        // mDisableMotor = mAbsEncoder.getZeroOffset() == 0;
        // if(mDisableMotor) 
        //     System.out.println("zero offset not applied, disabling");
        
        mAbsEncoder.setZeroOffset(motorConstants.kZeroOffset.getRotations()); //this is in rotation because the encoder's position conversion factor hasn't been set yet
        mAbsEncoder.setPositionConversionFactor(motorConstants.kPositionConversionFactor);
        // mAbsEncoder.setInverted(motorConstants.kInverted);
    }

    protected void pidControllerInit() {
        mPIDController = mMotor.getPIDController(); 
        
        mPIDController.setFeedbackDevice(mAbsEncoder);
        
        mPIDController.setP(kConstants.kPIDConstants.kP);
        mPIDController.setI(kConstants.kPIDConstants.kI);
        mPIDController.setD(kConstants.kPIDConstants.kD);
        mPIDController.setFF(kConstants.kPIDConstants.kFF);

        // mPIDController.setSmartMotionMaxAccel(kConstants.kSmartMotionsConstants.kSmartMotionMaxAccel, 0);
        // mPIDController.setSmartMotionMaxVelocity(kConstants.kSmartMotionsConstants.kSmartMotionMaxVelocity, 0);
        // mPIDController.setSmartMotionMinOutputVelocity(kConstants.kSmartMotionsConstants.kSmartMotionMinOutputVelocity, 0);
    }


    //unit conversions:
    public Rotation2d HorizonToNative(Rotation2d armRotation) {
        var outputRot = armRotation.unaryMinus().plus(kConstants.kHorizonDeflection);

        // To match the encoder, we want angles in [0, 2PI]
        var radValue = outputRot.getRadians();
        if (radValue < 0) {
            outputRot = Rotation2d.fromRadians(radValue + 2 * Math.PI);
        }

        return outputRot;
    }

    public Rotation2d nativeToHorizon(Rotation2d nativeRotation) {
        return nativeRotation.minus(kConstants.kHorizonDeflection).unaryMinus();
    }

    // getters for position:

    public Rotation2d getHorizonPosition() {
        return nativeToHorizon(getNativePosition());
    }

    public Rotation2d getNativePosition() {
        return Rotation2d.fromRadians(mAbsEncoder.getPosition());
    }

    //setters for position:
    public void setHorizonReference(Rotation2d ref) {
        setNativeReference(HorizonToNative(ref));
    }

    protected void setNativeReference(Rotation2d ref) {
        System.out.println("native set ref called");

        if(isNativeRefSafe(ref) && !mDisableMotor){
            currentRefrence = ref;
            setRef = true;
        } else {
            System.out.println("testing please say this is not the issue");
            onUnsafeNativeRef(ref);
        }
    }

    protected double calculateTrapazoidSetpoint(){
        System.out.println("calc trapazoid running");
        
        // double currVelocity = mAbsEncoder.getVelocity();
        // double mModeledPosition = getNativePosition().getRadians();
        var currState = new TrapezoidProfile.State(mModeledPosition, mModeledVelocity);
        var goalState = new TrapezoidProfile.State(currentRefrence.getRadians(), 0);
        TrapezoidProfile currMotionProfile = new TrapezoidProfile(kConstants.kTrapazoidConstraints, goalState, currState);
        System.out.println("vel: "+currMotionProfile.calculate(Arm.ticLength).velocity);
        State calcState = currMotionProfile.calculate(Arm.ticLength);
        mModeledVelocity = calcState.velocity;
        mModeledPosition = calcState.position;
        // By doing this, we are just playing back the motion profile in case the PID controller doesn't keep up.

        return calcState.position;

        // return currentRefrence.getRadians();
    }

    abstract protected boolean isNativeRefSafe(Rotation2d ref);

    abstract protected void onUnsafeNativeRef(Rotation2d ref);

    //periodic methodds:
    abstract protected double calculateFeedforwardValue();

    public void onPeriodic() {
        if (setRef)
            mPIDController.setReference(
                calculateTrapazoidSetpoint(), 
                ControlType.kPosition, 0, 
                calculateFeedforwardValue()
            );
        // System.out.println(currentRefrence.getDegrees() + " : " + calculateFeedforwardValue());
    }

    //getters for debug teleop:
    public CANSparkMax getMotor() {
        return mMotor;
    }

    public SparkMaxAbsoluteEncoder getAbsEncoder() {
        return mAbsEncoder;
    }

     /**
     * @return the refrence the PID controller is in native angles
     */
    public Rotation2d getCurrentReference() {
        return currentRefrence;
    }

}

