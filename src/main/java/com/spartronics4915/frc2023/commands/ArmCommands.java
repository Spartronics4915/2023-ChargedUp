package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.ArmSubsystem;
import com.spartronics4915.frc2023.subsystems.ArmSubsystem.ArmState;
// import com.spartronics4915.frc2023.subsystems.Arm.ArmState;
import com.spartronics4915.frc2023.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmCommands {
    private final ArmSubsystem mArm;
    
    public ArmCommands(ArmSubsystem mArm2) {
        mArm = mArm2;
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
