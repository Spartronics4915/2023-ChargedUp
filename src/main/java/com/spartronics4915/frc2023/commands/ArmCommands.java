package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.Arm;
import com.spartronics4915.frc2023.subsystems.Arm.ArmState;
// import com.spartronics4915.frc2023.subsystems.Arm.ArmState;
import com.spartronics4915.frc2023.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmCommands {
    private final Arm mArm;
    
    public ArmCommands(Arm arm) {
        mArm = arm;
    }

    public class SetArmState extends InstantCommand {
        public SetArmState(ArmState armState) {
            super(
                () -> {
                    mArm.setState(armState);
                },
                mArm
            );
        }
    }
}
