package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.ArmSubsystem;
import com.spartronics4915.frc2023.subsystems.ArmSubsystem.ArmState;
// import com.spartronics4915.frc2023.subsystems.Arm.ArmState;
import com.spartronics4915.frc2023.subsystems.Intake.IntakeState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import static com.spartronics4915.frc2023.Constants.Arm.Auto.*;
import static com.spartronics4915.frc2023.Constants.Arm.kArmRetractedPriorWaitDuration;;


public class ArmCommands {
    private final ArmSubsystem mArm;
	private final IntakeCommands mIntakeCommands;
    
    public ArmCommands(ArmSubsystem arm, IntakeCommands intakeCommands) {
        mArm = arm;
		mIntakeCommands = intakeCommands;
    }

    public class SetArmState extends SequentialCommandGroup {
        public SetArmState(ArmState armState) {
            addRequirements(mArm);
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

    public class SetArmLocalStateSimple extends CommandBase {
        ArmState mArmState;

        public SetArmLocalStateSimple(ArmState armState) {
            addRequirements(mArm);
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
            addRequirements(mArm);
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

    public class ResetCommand extends CommandBase{
        public ResetCommand(){
            mArm.clearReferences();
        }
    }

	public class PieceInteractCommand extends SequentialCommandGroup {
		public PieceInteractCommand(ArmState armState, IntakeState intakeState) {
			super(
				new SetArmState(armState),
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
			super(armState, IntakeState.CUBE_OUT);
		}
	}

    //marshall's code for sequenced arm

    public class WaitForPivotToArrive extends CommandBase{
        double mDegreesTolerance;
        Rotation2d mTargetAngle;

        public WaitForPivotToArrive(Rotation2d targetAngle, double degreesTolerance) {
            mDegreesTolerance = degreesTolerance;
            mTargetAngle = targetAngle;

        }

        @Override
        public boolean isFinished() {

            boolean finished;
            Rotation2d currPos = mArm.getPivot().getCurrentReference();
            if (Math.abs(currPos.minus(mTargetAngle).getDegrees()) < mDegreesTolerance) 
            {

                finished = true;
            }
            else {
                finished = false;
            }

            System.out.println("WaitforPivot FInished: "+ finished + " target:" + mTargetAngle + " currPos: " + currPos.getDegrees());
            return finished;
        }
    }

    public CommandBase getGoToPresetArmStatePivotFirstCommand(ArmState armState, boolean waitForExtender) {
        var untuckCommand = new UntuckWristIfNecessary().withTimeout(1);
        var seqCommands = new SequentialCommandGroup(
            // mIntakeCommands.getOutSpeedCommand(armState.outSpeed),
            untuckCommand,
            new SetArmLocalStateSimple(armState),
            new WaitForPivotToArrive(armState.armTheta, 5)
        );

        if(waitForExtender) {
            seqCommands = seqCommands.andThen(mArm.getExtender().extendToNInches(armState.armRadius));
        } else {
            seqCommands = seqCommands.andThen(new ScheduleCommand(mArm.getExtender().extendToNInches(armState.armRadius)));
        }
        return  seqCommands;
    }

    public CommandBase getGoToPresetArmStateExtendFirstCommand(ArmState armState, boolean waitForPivot) {
        var seqCommands = new SequentialCommandGroup(
            mArm.getExtender().extendToNInches(armState.armRadius),
            new SetArmLocalStateSimple(armState)
        );

        if(waitForPivot) {
            seqCommands = seqCommands.andThen(new WaitForPivotToArrive(armState.armTheta, 5));
        }
        return  seqCommands;
    }


    public class UntuckWristIfNecessary extends CommandBase {

        boolean mFinished;
        double kWristSafeAngle = 105;
        public UntuckWristIfNecessary() {
            mFinished = true;
        }

        @Override
        public void initialize() {
            var currLocalPosition = mArm.getLocalPosition();

            if(currLocalPosition.armTheta.getDegrees() < -40) {
                if(currLocalPosition.wristTheta.getDegrees() > kWristSafeAngle) {
                    mArm.getWrist().setHorizonReference(Rotation2d.fromDegrees(50));
                    mFinished = false;
                }
            }
        }

        @Override
        public boolean isFinished() {
            return mFinished || (mArm.getWrist().getCurrentReference().getDegrees() < kWristSafeAngle);
        }

    }
}
