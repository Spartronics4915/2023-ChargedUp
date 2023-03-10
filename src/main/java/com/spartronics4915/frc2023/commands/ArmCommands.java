package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.ArmSubsystem;
import com.spartronics4915.frc2023.subsystems.ArmSubsystem.ArmState;
// import com.spartronics4915.frc2023.subsystems.Arm.ArmState;
import com.spartronics4915.frc2023.subsystems.Intake.IntakeState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import static com.spartronics4915.frc2023.Constants.Arm.Auto.*;

import javax.lang.model.element.Element;

import static com.spartronics4915.frc2023.Constants.Arm.kArmRetractedPriorWaitDuration;;


public class ArmCommands {
    private final ArmSubsystem mArm;
	private final IntakeCommands mIntakeCommands;
    
    public ArmCommands(ArmSubsystem arm, IntakeCommands intakeCommands) {
        mArm = arm;
		mIntakeCommands = intakeCommands;
    }

    public class SetArmStateComplex extends SequentialCommandGroup {
        public SetArmStateComplex(ArmState armState) {
            if (mArm.getDesiredGlobalState() == ArmState.RETRACTED || armState == ArmState.RETRACTED)
                addCommands(
                    new InstantCommand(() -> {
                        mArm.setDesiredGlobalState(ArmState.RETRACTED_PRIOR);
                    }, mArm),
                    new WaitCommand(kArmRetractedPriorWaitDuration),
                    new InstantCommand(() -> {
                        mArm.setDesiredGlobalState(armState);
                    }, mArm)
                );
            else addCommands(
                new InstantCommand(() -> {
                    mArm.setDesiredGlobalState(armState);
                }, mArm)
            );
        }
    }

    public class SetArmLocalState extends CommandBase {
        ArmState mArmState;

        public SetArmLocalState(ArmState armState) {
            mArmState = armState;
        }

        @Override
        public void execute()
        {
            mArm.setDesiredLocalState(mArmState);

        }
        @Override
        public boolean isFinished(){
            return true;
        }
    }


    public class TransformArmState extends CommandBase {
        private double mExtensionDelta;
        private Rotation2d mArmDelta, mWristDelta;

        public TransformArmState(double extensionDelta, Rotation2d armDelta, Rotation2d wristDelta) {
            mExtensionDelta = extensionDelta;
            mArmDelta = armDelta;
            mWristDelta = wristDelta;
        }

        @Override
        public void execute() {
            mArm.transformPosition(mExtensionDelta, mArmDelta, mWristDelta);
        }

        @Override
        public boolean isFinished()
        {
            return false;
        }
    }

	public class PieceInteractCommand extends SequentialCommandGroup {
		public PieceInteractCommand(ArmState armState, IntakeState intakeState) {
			super(
				new SetArmLocalState(armState),
				new WaitCommand(kArmStateChangeDuration),
				mIntakeCommands.new SetIntakeState(intakeState),
				new WaitCommand(kGrabDuration),
				mIntakeCommands.new SetIntakeState(IntakeState.OFF)
			);
		}
	}

	public class GrabPiece extends PieceInteractCommand {
		public GrabPiece(ArmState armState) {
			super(armState, IntakeState.IN);
		}
	}
	
	public class ReleasePiece extends PieceInteractCommand {
		public ReleasePiece(ArmState armState) {
			super(armState, IntakeState.OUT);
		}
	}

    public class WaitForPivotToArrive extends CommandBase{
        double mDegreesTolerance;
        Rotation2d mTargetAngle;

        public WaitForPivotToArrive(Rotation2d targetAngle, double degreesTolerance) {
            mDegreesTolerance = degreesTolerance;
            mTargetAngle = targetAngle;

        }

        @Override
        public boolean isFinished() {

            Rotation2d currPos = mArm.getPivot().getModeledPosition();
            if (Math.abs(currPos.minus(mTargetAngle).getDegrees()) < mDegreesTolerance) 
            {
                return true;
            }
            else {
                return false;
            }
        }
    }
}
