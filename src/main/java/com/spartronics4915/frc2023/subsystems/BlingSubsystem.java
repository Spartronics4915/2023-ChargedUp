package com.spartronics4915.frc2023.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2023.Constants.Bling.*;

import com.spartronics4915.frc2023.bling.CustomLEDPattern;

public class BlingSubsystem extends SubsystemBase {
    private final AddressableLED mLEDUnderglow;
    private final AddressableLEDBuffer mLEDUnderglowBuffer;

    private final SendableChooser<CustomLEDPattern> mBlingSelector;
    private CustomLEDPattern mCurrentPattern;

    private static final BlingSubsystem mInstance = new BlingSubsystem();

    private BlingSubsystem() {
        mLEDUnderglow = new AddressableLED(kLEDUnderglowPort);
        mLEDUnderglowBuffer = new AddressableLEDBuffer(kLEDUnderglowBufferLength);
        mLEDUnderglow.setLength(mLEDUnderglowBuffer.getLength());

        mBlingSelector = new SendableChooser<>();
        configureBlingSelector();

        mCurrentPattern = kBlingPatterns[kDefaultBlingPatternIndex].pattern;
    }

    public static BlingSubsystem getInstance() {
        return mInstance;
    }

    private void configureBlingSelector() {
        for (BlingPattern entry : kBlingPatterns)
            mBlingSelector.addOption(entry.name, entry.pattern);
        mBlingSelector.setDefaultOption(
            kBlingPatterns[kDefaultBlingPatternIndex].name, 
            kBlingPatterns[kDefaultBlingPatternIndex].pattern);
        
        SmartDashboard.putData("Bling", mBlingSelector);
    }

    public void startUnderglow() {
        mLEDUnderglow.start();
    }

    public void stopUnderglow() {
        mLEDUnderglow.stop();
    }

    public void setUnderglowPattern(CustomLEDPattern pattern) {
        pattern.setLEDs(mLEDUnderglowBuffer);
    }

    public CommandBase setUnderglowPatternCommand(CustomLEDPattern pattern) {
        return runOnce(() -> setUnderglowPattern(pattern));
    }

    @Override
    public void periodic() {
        if (mCurrentPattern.isAnimated()) {
            mCurrentPattern.setLEDs(mLEDUnderglowBuffer);
        }
        
        final var selected = mBlingSelector.getSelected();
        if (selected != mCurrentPattern) {
            mCurrentPattern = selected;
            mCurrentPattern.setLEDs(mLEDUnderglowBuffer);
        }

        mLEDUnderglow.setData(mLEDUnderglowBuffer);
    }
}
