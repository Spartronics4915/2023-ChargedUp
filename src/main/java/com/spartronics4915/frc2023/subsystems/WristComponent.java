package com.spartronics4915.frc2023.subsystems;

import com.spartronics4915.frc2023.Constants.Arm;
import com.spartronics4915.frc2023.subsystems.ArmJointAbstractComponent;

import edu.wpi.first.math.geometry.Rotation2d;

public class WristComponent extends ArmJointAbstractComponent {

    private PivotComponent mPivotMotor;

    public WristComponent(PivotComponent pivotMotor) {
        super(Arm.kWristMotorConstants);
        mPivotMotor = pivotMotor;
    }


    private Rotation2d getAngleWithEarth() {
        Rotation2d pivotArmAngle = mPivotMotor.getHorizonPosition();
        Rotation2d wristArmAngle = getHorizonPosition();
        Rotation2d wristAngleWithEarth = pivotArmAngle.plus(wristArmAngle);
        return wristAngleWithEarth;
    }

    @Override
    protected double calculateFeedforwardValue() {
        return Math.cos(getAngleWithEarth().getRadians());
    }

    @Override
    protected boolean isNativeRefSafe(Rotation2d ref) {
        return true;
    }

    @Override
    protected void onUnsafeNativeRef(Rotation2d ref) {
        System.out.println("unsafe ref, not setting");
    }
}
