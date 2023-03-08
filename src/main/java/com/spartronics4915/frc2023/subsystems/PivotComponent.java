package com.spartronics4915.frc2023.subsystems;

import com.spartronics4915.frc2023.Constants.Arm;
import com.spartronics4915.frc2023.subsystems.ArmJointAbstractSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotComponent extends ArmJointAbstractSubsystem {
    
    public PivotComponent() {
        super(Arm.kPivotMotorConstants);
    }

    @Override
    protected double calculateFeedforwardValue() {
        return Math.cos(getHorizonPosition().getRadians());
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
