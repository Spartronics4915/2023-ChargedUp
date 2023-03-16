package com.spartronics4915.frc2023.subsystems;

import com.spartronics4915.frc2023.Constants.Arm;
import com.spartronics4915.frc2023.subsystems.ArmJointAbstractComponent;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotComponent extends ArmJointAbstractComponent {
    
    public PivotComponent() {
        super(Arm.kPivotMotorConstants);
    }

    @Override
    protected double calculateFeedforwardValue() {
        return Math.cos(getHorizonPosition().getRadians());
    }

    @Override
    protected boolean isNativeRefSafe(Rotation2d ref) {
        Rotation2d horizon = nativeToHorizon(ref);
        return (horizon.getDegrees() > kConstants.kMinRotation.getDegrees()) && (horizon.getDegrees() < kConstants.kMaxRotation.getDegrees());
    }

    @Override
    protected void onUnsafeNativeRef(Rotation2d ref) {
        System.out.println("unsafe ref, not setting ref: " + nativeToHorizon(ref));
    }
}
