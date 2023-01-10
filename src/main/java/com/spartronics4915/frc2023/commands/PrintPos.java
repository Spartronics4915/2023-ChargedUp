package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.subsystems.ExampleSubsystem;
import com.spartronics4915.frc2023.PhotonCameraWrapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class PrintPos extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private PhotonCameraWrapper cam;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public PrintPos(PhotonCameraWrapper cameraWrapper) {
        cam = cameraWrapper;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d result = cam.getEstimatedGlobalPose(new Pose2d()).getFirst();
        Double[] split = {result.getX(), result.getY()};
        SmartDashboard.putNumberArray("Pose2D", split);
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
