// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2023;

import com.spartronics4915.frc2023.commands.DebugTeleopCommands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private RobotContainer mRobotContainer;

    private Command mAutonomousCommand;
    private Command mTeleopInitCommand;
    private Command mTestingCommand;

    private AddressableLED mLEDUnderglow;
    private AddressableLEDBuffer mLEDUnderglowBuffer;
    private int mRainbowFirstPixelHue;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        mRobotContainer = new RobotContainer();

        mAutonomousCommand = mRobotContainer.getAutonomousCommand();
        mTeleopInitCommand = mRobotContainer.getTeleopInitCommand();
        mTestingCommand = mRobotContainer.getTestingCommand();

        mRobotContainer.initRobot();

        mLEDUnderglow = new AddressableLED(0);

        mLEDUnderglowBuffer = new AddressableLEDBuffer(120);
        mLEDUnderglow.setLength(mLEDUnderglowBuffer.getLength());
        mLEDUnderglow.setData(mLEDUnderglowBuffer);
        mLEDUnderglow.start();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        
        var alliance = DriverStation.getAlliance();

        if (alliance == Alliance.Red){
            for (int i = 0; i < mLEDUnderglowBuffer.getLength(); i++)
                mLEDUnderglowBuffer.setRGB(i, 255, 0, 0);
        } else if (alliance == Alliance.Blue){
            for (int i = 0; i < mLEDUnderglowBuffer.getLength(); i++)
                mLEDUnderglowBuffer.setRGB(i, 0, 0, 255);
        } else {rainbow();}

    mLEDUnderglow.setData(mLEDUnderglowBuffer);
    }

    private void rainbow() {
        
        for (var i = 0; i < mLEDUnderglowBuffer.getLength(); i++) {
        
          final var hue = (mRainbowFirstPixelHue + (i * 180 / mLEDUnderglowBuffer.getLength())) % 180;
          
          mLEDUnderglowBuffer.setHSV(i, hue, 255, 128);
        }
        mRainbowFirstPixelHue += 4;
        
        mRainbowFirstPixelHue %= 180;
      }


    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {

        mAutonomousCommand = mRobotContainer.getAutonomousCommand();
        mRobotContainer.resetForAuto();
        // schedule the autonomous command (example)
        if (mAutonomousCommand != null) {
            mAutonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.

        mRobotContainer.resetForTeleop();
        if (mAutonomousCommand != null) {
            mAutonomousCommand.cancel();
        }
        
        if (mTeleopInitCommand != null) {
            mTeleopInitCommand.schedule();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        if (mTestingCommand != null) {
            mTestingCommand.schedule();
        }
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
