package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.Intake;
import com.spartronics4915.frc2023.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeCommands {
    private final Intake mIntake;

    public IntakeCommands(Intake intake) {
        mIntake = intake;
    }

    public class SetIntakeState extends InstantCommand {
        public SetIntakeState(IntakeState state) {
            super(
                () -> {
                    System.out.println(state);
                    mIntake.setState(state);
                },
                mIntake
            );
            addRequirements(mIntake);
        }
    }
}
