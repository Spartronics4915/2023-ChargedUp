package com.spartronics4915.frc2023.commands;

import org.photonvision.PhotonCamera;

import com.spartronics4915.frc2023.PhotonCameraWrapper;
import com.spartronics4915.frc2023.PhotonCameraWrapper.VisionMeasurement;
import com.spartronics4915.frc2023.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class PrintPos extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Swerve mSwerve;
    VisionMeasurement result;
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
        // System.out.println("epic test print");
        // VisionMeasurement result = mSwerve.mCameraWrapper.getEstimatedGlobalPose();
        // if(result != null) {
        //     System.out.println("(" + result.mPose.getX() + "," + result.mPose.getY() + ")");
        //     SmartDashboard.putNumber("Pose2D X", result.mPose.getX());
        //     SmartDashboard.putNumber("Pose2D Y", result.mPose.getY());
        // }
        result = mSwerve.mCameraWrapper.getEstimatedGlobalPose();
        if(result != null) {
            mSwerve.setPose(result.mPose);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("epic test print");
        result = mSwerve.mCameraWrapper.getEstimatedGlobalPose();
        if(result != null) {
            System.out.println("(" + result.mPose.getX() + "," + result.mPose.getY() + ")");
            SmartDashboard.putNumber("Vision X", result.mPose.getX());
            SmartDashboard.putNumber("Vision Y", result.mPose.getY());
            SmartDashboard.putNumber("Vision Rotation", result.mPose.getRotation().getRadians());
        }
        SmartDashboard.putNumber("Swerve X", mSwerve.getPose().getX());
        SmartDashboard.putNumber("Swerve Y", mSwerve.getPose().getY());
        SmartDashboard.putNumber("Swerve Rotation", mSwerve.getPose().getRotation().getRadians());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
