package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.Swerve;
import com.spartronics4915.frc2023.subsystems.SwerveModule;
import com.spartronics4915.frc2023.subsystems.ArmSubsystem.ArmPosition;
import com.spartronics4915.frc2023.subsystems.ArmSubsystem.ArmState;
import com.spartronics4915.frc2023.subsystems.ArmSubsystem;
import com.spartronics4915.frc2023.subsystems.ExtenderSubsystem;
import com.spartronics4915.frc2023.subsystems.MotorAbsEncoderComboSubsystem;
import com.spartronics4915.frc2023.commands.SwerveCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Map;

public final class DebugTeleopCommands {

    public static void teleopInit(Swerve swerve_subsystem) {
        swerve_subsystem.resetToAbsolute();
        swerve_subsystem.resetYaw();
        swerve_subsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))); // for odometry testing
        swerve_subsystem.stop();
        swerve_subsystem.alignModules();
        System.out.println("Teleopinit called");

    }

    public static class ChassisWidget {
        private GenericEntry yawEntry;

        ChassisWidget(ShuffleboardTab tab) {
            ShuffleboardLayout yawLayout = tab.getLayout("Chassis", BuiltInLayouts.kList)
                    .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

            yawEntry = yawLayout.add("Yaw (Degrees)", 0).getEntry();
        }

        public void update(Swerve swerveSubsystem) {
            yawEntry.setDouble(swerveSubsystem.getYaw().getDegrees());
        }
    }

    public static class SwerveModuleWidget {
        private GenericEntry angleEntry;
        private GenericEntry state_angle, abs_encoder, rel_encoder, rel_encoder_deg, shifted_abs_encoder;

        SwerveModuleWidget(ShuffleboardTab tab, String name) {
            ShuffleboardLayout swerve_module = tab.getLayout(name, BuiltInLayouts.kList)
                    .withSize(2, 2).withProperties(Map.of("Label position", "LEFT"));

            angleEntry = swerve_module.add("desired.angle", 0).getEntry();
            state_angle = swerve_module.add("current.angle", 0).getEntry();
            abs_encoder = swerve_module.add("abs_encoder", 0).getEntry();
            rel_encoder = swerve_module.add("rel_encoder", 0).getEntry();
            rel_encoder_deg = swerve_module.add("rel_encoder (degrees)", 0).getEntry();
            shifted_abs_encoder = swerve_module.add("shifted abs_encoder", 0).getEntry();
        }

        public void update(SwerveModule module) {
            SwerveModuleState current = module.getState();
            SwerveModuleState desired = module.getDesiredState();

            angleEntry.setDouble(desired.angle.getDegrees());
            state_angle.setDouble(current.angle.getDegrees());
            abs_encoder.setDouble(module.getAbsoluteEncoderValue());
            rel_encoder.setDouble(module.getRelativeEncoderValue());
            rel_encoder_deg.setDouble(Rotation2d.fromRadians(module.getRelativeEncoderValue()).getDegrees());
            shifted_abs_encoder.setDouble(module.getShiftedAbsoluteEncoderRotations());

        }
    }

    public static class SwerveTab {
        SwerveModuleWidget module0, module1, module2, module3;
        ChassisWidget chassisWidget;
        ShuffleboardTab tab;
        Swerve swerve_subsystem;
        SwerveCommands mSwerveCommands;

        SwerveTab(Swerve swerve, SwerveCommands swerveCommands) {
            mSwerveCommands = swerveCommands;
            tab = Shuffleboard.getTab("Swerve");
            // module0 = new SwerveModuleWidget(tab, "Module 0");
            // module1 = new SwerveModuleWidget(tab, "Module 1");
            // module2 = new SwerveModuleWidget(tab, "Module 2");
            // module3 = new SwerveModuleWidget(tab, "Module 3");
            chassisWidget = new ChassisWidget(tab);

            swerve_subsystem = swerve;
            ShuffleboardLayout elevatorCommands = tab.getLayout("Orientation", BuiltInLayouts.kList)
                    .withSize(2, 3)
                    .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem,
            // Rotation2d.fromDegrees(0)).withName("Orientation 0"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem,
            // Rotation2d.fromDegrees(90)).withName("Orientation 90"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem,
            // Rotation2d.fromDegrees(180)).withName("Orientation 180"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem,
            // Rotation2d.fromDegrees(270)).withName("Orientation 270"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem,
            // Rotation2d.fromDegrees(360)).withName("Orientation 360"));

            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem,
            // Rotation2d.fromDegrees(-90)).withName("Orientation -90"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem,
            // Rotation2d.fromDegrees(-180)).withName("Orientation -180"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem,
            // Rotation2d.fromDegrees(-270)).withName("Orientation -270"));
            elevatorCommands
                    .add(Commands.runOnce(() -> swerve_subsystem.resetToAbsolute()).withName("Reset to Absolute"));
            elevatorCommands.add(mSwerveCommands.new RotateToYaw(Rotation2d.fromDegrees(45)).withName("Rotate 45"));
        }

        public void update() {

            // var swerve_modules = swerve_subsystem.getSwerveModules();

            // module0.update(swerve_modules[0]);
            // module1.update(swerve_modules[1]);
            // module2.update(swerve_modules[2]);
            // module3.update(swerve_modules[3]);

            chassisWidget.update(swerve_subsystem);
        }
    }

    public static class ExtenderWidget {
        private GenericEntry extenderPos, targetRef;
        private GenericEntry PIDSpeed;

        ExtenderWidget(ShuffleboardTab tab) {
            ShuffleboardLayout module = tab.getLayout("Extender", BuiltInLayouts.kList).withSize(2, 3)
                    .withProperties(Map.of("Label position", "LEFT"));
            extenderPos = module.add("ExtenderPos", 0).getEntry();
            PIDSpeed = module.add("PID Speed", 0).getEntry();
            targetRef = module.add("Target Reference", 0).getEntry();
        }

        public void update(ExtenderSubsystem subsystem) {
            extenderPos.setDouble(subsystem.getPosition());
            PIDSpeed.setDouble(subsystem.getMotor().getAppliedOutput());
            targetRef.setDouble(subsystem.getReference());
        }
    }

    public static class PowerWidget {
        private GenericEntry totalCurrent;
        private PowerDistribution powerPanel;

        public PowerWidget(ShuffleboardTab tab) {
            ShuffleboardLayout layout = tab.getLayout("Power", BuiltInLayouts.kList).withSize(2, 3)
                    .withProperties(Map.of("Label position", "LEFT"));
            powerPanel = new PowerDistribution(1, ModuleType.kRev);

            totalCurrent = layout.add("totalPower", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
        }

        public void update() {
            totalCurrent.setDouble(powerPanel.getTotalCurrent());
        }
    }

    public static class ArmWidget {
        // private GenericEntry linActDistance,stateRadius;
        private GenericEntry wristLeveledRotation, stateWristLeveledRotation, wristSpeed;
        private GenericEntry shoulderRaw,  shoulderNative, shoulderArm, stateShoulderRotation, refShoulderRotation, pivotSpeed;
        private GenericEntry shoulderArmPlus30Native, shouldArmMinus30Native;
        private GenericEntry wristRaw,  wristNative, wristArm, statewristRotation, refwristRotation;
        private GenericEntry wristArmPlus30Native, wristArmMinus30Native;
        ArmWidget(ShuffleboardTab tab, String name) {
            ShuffleboardLayout armModule = tab.getLayout(name, BuiltInLayouts.kList).withSize(2, 3)
                    .withProperties(Map.of("Label position", "LEFT"));

            ShuffleboardLayout wristLayout = tab.getLayout("Wrist", BuiltInLayouts.kList).withSize(2, 3)
            .withProperties(Map.of("Label position", "LEFT"));
            // linActDistance = armModule.add("current radius", 0).getEntry();
            // stateRadius = armModule.add("desired radius",0).getEntry();


            wristRaw = wristLayout.add("Wrist (Raw)", 0).getEntry();
            wristNative = wristLayout.add("Wrist (Native)", 0).getEntry();
            wristArm = wristLayout.add("Wrist (Arm)", 0).getEntry();
            wristArmMinus30Native = wristLayout.add("Wrist Native w Arm at -30", 0).getEntry();
            wristArmPlus30Native = wristLayout.add("Wrist Native w Arm at +30", 0).getEntry();

            shoulderRaw = armModule.add("Shoulder (Raw)", 0).getEntry();
            shoulderNative = armModule.add("Shoulder (Native)", 0).getEntry();
            shoulderArm = armModule.add("Shoulder (Arm)", 0).getEntry();
            shouldArmMinus30Native = armModule.add("Native w Arm at -30", 0).getEntry();
            shoulderArmPlus30Native = armModule.add("Native w Arm at +30", 0).getEntry();
            
            stateShoulderRotation = armModule.add("desired shoulder angle", 0).getEntry();
            refShoulderRotation = armModule.add("current refrence", 0).getEntry();
            pivotSpeed = armModule.add("pivot speed", 0).getEntry();
        }

        public void update(ArmSubsystem module) {
            ArmPosition current = module.getPosition();
            ArmState desired = module.getState();
            MotorAbsEncoderComboSubsystem[] motors = module.getMotors();
            // linActDistance.setDouble(current.armRadius);
            // stateRadius.setDouble(desired.armRadius);


            wristRaw.setDouble(module.getPivot().getRawPosition());
            wristNative.setDouble(module.getPivot().getNativePosition().getDegrees());
            wristArm.setDouble(module.getPivot().getArmPosition().getDegrees());
            wristArmMinus30Native.setDouble(module.getPivot().armToNative(Rotation2d.fromDegrees(-30)).getDegrees());
            wristArmPlus30Native.setDouble(module.getPivot().armToNative(Rotation2d.fromDegrees(30)).getDegrees());

            shoulderRaw.setDouble(module.getPivot().getRawPosition());
            shoulderNative.setDouble(module.getPivot().getNativePosition().getDegrees());
            shoulderArm.setDouble(module.getPivot().getArmPosition().getDegrees());
            shouldArmMinus30Native.setDouble(module.getPivot().armToNative(Rotation2d.fromDegrees(-30)).getDegrees());
            shoulderArmPlus30Native.setDouble(module.getPivot().armToNative(Rotation2d.fromDegrees(30)).getDegrees());
            stateShoulderRotation.setDouble(desired.armTheta.getDegrees());
            refShoulderRotation.setDouble(module.getRef().getDegrees());
            pivotSpeed.setDouble(motors[0].getMotorSpeed());
        }
    }

    public static class ArmTab {
        ArmWidget widget0;
        ExtenderWidget extenderWidget;
        ShuffleboardTab tab;
        ArmSubsystem mArmSubsystem;
        ArmCommands mArmCommands;
        PowerWidget powerWidget;

        ArmTab(ArmSubsystem armSubsystem, ArmCommands armCommands) {
            mArmCommands = armCommands;
            tab = Shuffleboard.getTab("Arm Tab");
            // module0 = new SwerveModuleWidget(tab, "Module 0");
            // module1 = new SwerveModuleWidget(tab, "Module 1");
            // module2 = new SwerveModuleWidget(tab, "Module 2");
            // module3 = new SwerveModuleWidget(tab, "Module 3");
            widget0 = new ArmWidget(tab, "arm widget");
            extenderWidget = new ExtenderWidget(tab);

            powerWidget = new PowerWidget(tab);
            mArmSubsystem = armSubsystem;
            ShuffleboardLayout elevatorCommands = tab.getLayout("Orientation", BuiltInLayouts.kList)
                    .withSize(2, 3)
                    .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem,
            // Rotation2d.fromDegrees(0)).withName("Orientation 0"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem,
            // Rotation2d.fromDegrees(90)).withName("Orientation 90"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem,
            // Rotation2d.fromDegrees(180)).withName("Orientation 180"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem,
            // Rotation2d.fromDegrees(270)).withName("Orientation 270"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem,
            // Rotation2d.fromDegrees(360)).withName("Orientation 360"));

            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem,
            // Rotation2d.fromDegrees(-90)).withName("Orientation -90"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem,
            // Rotation2d.fromDegrees(-180)).withName("Orientation -180"));
            // elevatorCommands.add(SimpleAutos.forceOrientation(swerve_subsystem,
            // Rotation2d.fromDegrees(-270)).withName("Orientation -270"));
            elevatorCommands.add((mArmCommands.new SetArmState(ArmState.ARM_LEVEL)).withName("LEVEL"));
            elevatorCommands.add((mArmCommands.new SetArmState(ArmState.ARM_HIGH)).withName("HIGH"));
            elevatorCommands.add((mArmCommands.new SetArmState(ArmState.ARM_LOW)).withName("LOW"));
            elevatorCommands.add(new ExtenderCommands.ExtendNInches(3,mArmSubsystem.getExtender()).withName("Extend 3 Inches"));
            elevatorCommands.add(new ExtenderCommands.ExtendNInches(-3,mArmSubsystem.getExtender()).withName("Extend -3 Inches"));
            elevatorCommands.add(Commands.runOnce(() -> mArmSubsystem.getExtender().setZero()).withName("Zero Encoder"));

        }

        public void update() {

            // var swerve_modules = swerve_subsystem.getSwerveModules();

            // module0.update(swerve_modules[0]);
            // module1.update(swerve_modules[1]);
            // module2.update(swerve_modules[2]);
            // module3.update(swerve_modules[3]);

            widget0.update(mArmSubsystem);
            extenderWidget.update(mArmSubsystem.getExtender());
            powerWidget.update();
        }
    }

    public static class ShuffleboardUpdateCommand extends CommandBase {

        ArmSubsystem mArmSubsystem;
        ArmTab mArmTab;
        ArmCommands mArmCommands;

        public ShuffleboardUpdateCommand(ArmSubsystem ArmSubsystem, ArmCommands ArmCommands) {
            mArmSubsystem = ArmSubsystem;
            mArmCommands = ArmCommands;
        }
        // Called when the command is initially scheduled.

        @Override
        public void initialize() {

            mArmTab = new ArmTab(mArmSubsystem, mArmCommands);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            mArmTab.update();
        }

        // Lets this command run even when disabled
        @Override
        public boolean runsWhenDisabled() {

            return true;
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return false;
        }

    }
}