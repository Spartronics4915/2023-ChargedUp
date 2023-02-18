package com.spartronics4915.frc2023.commands;

import org.photonvision.PhotonCamera;

import com.spartronics4915.frc2023.PhotonCameraWrapper;
import com.spartronics4915.frc2023.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class PrintPos extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Swerve mSwerve;
    private PhotonCameraWrapper cameraWrapper;
    // private PhotonCamera mPhotonCamera;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public PrintPos() {
        mSwerve = Swerve.getInstance();
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        while(true) {
            Pose2d result = mSwerve.mCameraWrapper.getEstimatedGlobalPose().getFirst();
            if(result != null) {
                SmartDashboard.putNumber("Pose2D X", result.getX());
                SmartDashboard.putNumber("Pose2D Y", result.getY());
        }
        }
        
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
