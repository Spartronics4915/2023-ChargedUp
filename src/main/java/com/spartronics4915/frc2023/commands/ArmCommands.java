package com.spartronics4915.frc2023.commands;

import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;
import com.spartronics4915.frc2023.Constants.Arm.ArmPositionConstants.ArmSettingsConstants;
import com.spartronics4915.frc2023.subsystems.ArmSubsystem;
import com.spartronics4915.frc2023.subsystems.ArmSubsystem.ArmState;
// import com.spartronics4915.frc2023.subsystems.Arm.ArmState;
import com.spartronics4915.frc2023.subsystems.Intake.IntakeState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static com.spartronics4915.frc2023.Constants.Arm.Auto.*;

import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

import javax.lang.model.element.Element;

import static com.spartronics4915.frc2023.Constants.Arm.kArmRetractedPriorWaitDuration;;


public class ArmCommands {
    private final ArmSubsystem mArm;
	private final IntakeCommands mIntakeCommands;
    private HashSet<Subsystem> mArmReq;

    public ArmCommands(ArmSubsystem arm, IntakeCommands intakeCommands) {
        mArm = arm;
		mIntakeCommands = intakeCommands;
        mArmReq = new HashSet<Subsystem>();
        mArmReq.add(mArm);
    }

    public class SetArmPivotWristLocalState extends CommandBase {
        ArmState mArmState;

        public SetArmPivotWristLocalState(ArmState armState) {
            mArmState = armState;
        }

        @Override
        public void initialize()
        {
            System.out.println("SetArmPivotWristLocalState: " + mArmState);
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
				getGoToPresetArmStatePivotFirstCommand(armState, true),
                new WaitCommand(1.25),
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

            boolean finished;
            Rotation2d currPos = mArm.getPivot().getModeledPosition();
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

        @Override
        public Set<Subsystem> getRequirements() {
            return mArmReq;
        }
    }

    public Command getGoToPresetArmStateSimultaneousCommand(ArmState armState) {
        var untuckCommand = new UntuckWristIfNecessary().withTimeout(1);
        var commands = new ParallelCommandGroup(
            mIntakeCommands.getOutSpeedCommand(armState.outSpeed),
            new SetArmPivotWristLocalState(armState),
            mArm.getExtender().setTargetCommandRunOnce(armState.armRadius)
        );
        return untuckCommand.andThen(commands);
    }

    public CommandBase getGoToPresetArmStatePivotFirstCommand(ArmState armState, boolean waitForExtender) {
        var untuckCommand = new UntuckWristIfNecessary().withTimeout(1);
        var seqCommands = new SequentialCommandGroup(
            mIntakeCommands.getOutSpeedCommand(armState.outSpeed),
            untuckCommand,
            new SetArmPivotWristLocalState(armState),
            new WaitForPivotToArrive(armState.armTheta, 5),
            mArm.getExtender().setTargetCommandRunOnce(armState.armRadius)
        );

        if(waitForExtender) {
            seqCommands = seqCommands.andThen(mArm.getExtender().waitForModeledExtenderToArriveCommand().withTimeout(3));
        } 
        return  seqCommands;
    }

    public CommandBase getGoToPresetArmStateExtendFirstCommand(ArmState armState, boolean waitForPivot) {
        var seqCommands = new SequentialCommandGroup(mArm.getExtender().setTargetCommandRunOnce(armState.armRadius),
                                                    mArm.getExtender().waitForModeledExtenderToArriveCommand().withTimeout(3), 
                                                    new SetArmPivotWristLocalState(armState)
        );

        if(waitForPivot) {
            seqCommands = seqCommands.andThen(new WaitForPivotToArrive(armState.armTheta, 5));
        }
        return  seqCommands;
    }

    public Command getTuckCommand() {
        return new SequentialCommandGroup(
            getGoToPresetArmStateSimultaneousCommand(ArmState.TUCK_INTERMEDIATE)
                .alongWith(mArm.getExtender().zeroExtenderCommand()),
            getGoToPresetArmStateSimultaneousCommand(ArmState.RETRACTED)
        );
    }

    public Command getGoToFloorCommand() {
        return getUntuckArmIfNecessaryCommand()
            .andThen(getGoToPresetArmStateSimultaneousCommand(ArmState.FLOOR_POS));
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
                    mArm.getWrist().setArmReference(Rotation2d.fromDegrees(50));
                    mFinished = false;
                }
            }
        }

        @Override
        public boolean isFinished() {
            return mFinished || (mArm.getWrist().getModeledPosition().getDegrees() < kWristSafeAngle);
        }

    }

    public Command getUntuckArmIfNecessaryCommand() {
        return new ParallelCommandGroup(
            new UntuckWristIfNecessary(),
            new ConditionalCommand(
                mArm.getExtender().zeroExtenderCommand()
                    .andThen(getGoToPresetArmStatePivotFirstCommand(ArmState.TUCK_INTERMEDIATE, false)),
                Commands.none(),
                () -> mArm.getPivot().getArmPosition().getDegrees() < -20
            )
        );
    }

    public CommandBase waitPivotPassAngleUp(double angDegrees) {

        return new WaitUntilCommand(()->mArm.getPivot().getModeledPosition().getDegrees() > angDegrees);

    }

    public CommandBase getTuckWristCommand() {
        // A tucked wrist means that it is turned up high 

        var wrist = mArm.getWrist();
        Rotation2d tuckAngle = Rotation2d.fromDegrees(105);
        CommandBase tuckCmd = mArm.runOnce(()->wrist.setArmReference(tuckAngle));
        BooleanSupplier checkCond = ()->(wrist.getModeledPosition().getDegrees() > 90);
        CommandBase waitCmd = new WaitUntilCommand(checkCond);
        return tuckCmd.andThen(waitCmd.withTimeout(2));

    }

    public CommandBase getRaiseToHighConeOverTopCommand() {

        CommandBase seqCmd = new SequentialCommandGroup(
        new UntuckWristIfNecessary(),
        // Go to 30 degrees up, no extension
        getGoToPresetArmStatePivotFirstCommand(ArmState.PRE_OVER_TOP_POSITION, false),
        // Extend to 3 inches out, don't wait.  This gets the arm out of the sticky zone.
        mArm.getExtender().setTargetCommandRunOnce(3),
        // Make sure the wrist is tucked to minimize torsion
        getTuckWristCommand(),
        // Now go to high cone spot
        getGoToPresetArmStatePivotFirstCommand(ArmState.CONE_LEVEL_2, true)
        );

        return seqCmd;
    }
}
