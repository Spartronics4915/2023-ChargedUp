package com.spartronics4915.frc2023.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2023.Constants.Bling.*;

import com.spartronics4915.frc2023.bling.CustomLEDPattern;
import com.spartronics4915.frc2023.bling.MatchAlliancePattern;

public class BlingSubsystem extends SubsystemBase {
    private final AddressableLED mLEDUnderglow;
    private final AddressableLEDBuffer mLEDUnderglowBuffer;
    
    private final AddressableLED mLEDDiagonal;
    private final AddressableLEDBuffer mLEDDiagonalBuffer;

    private final AddressableLED mLEDMast;
    private final AddressableLEDBuffer mLEDMastBuffer;

    private final SendableChooser<CustomLEDPattern> mUnderglowPatternSelector;
    private CustomLEDPattern mCurrentUnderglowPattern;

    private static final BlingSubsystem mInstance = new BlingSubsystem();

    private BlingSubsystem() {
        mLEDUnderglow = new AddressableLED(kLEDUnderglowPort);
        mLEDUnderglowBuffer = new AddressableLEDBuffer(kLEDUnderglowBufferLength);
        mLEDUnderglow.setLength(mLEDUnderglowBuffer.getLength());
        
        mLEDDiagonal = new AddressableLED(kLEDDiagonalPort);
        mLEDDiagonalBuffer = new AddressableLEDBuffer(kLEDDiagonalBufferLength);
        mLEDDiagonal.setLength(mLEDDiagonalBuffer.getLength());
        
        mLEDMast = new AddressableLED(kLEDMastPort);
        mLEDMastBuffer = new AddressableLEDBuffer(kLEDMastBufferLength);
        mLEDMast.setLength(mLEDMastBuffer.getLength());

        mUnderglowPatternSelector = new SendableChooser<>();
        configureBlingSelector();

        mCurrentUnderglowPattern = kBlingPatterns[kDefaultBlingPatternIndex].pattern;
    }

    public static BlingSubsystem getInstance() {
        return mInstance;
    }

    private void configureBlingSelector() {
        for (BlingPattern entry : kBlingPatterns) {
            mUnderglowPatternSelector.addOption(entry.name, entry.pattern);
        }
        mUnderglowPatternSelector.setDefaultOption(
            kBlingPatterns[kDefaultBlingPatternIndex].name, 
            kBlingPatterns[kDefaultBlingPatternIndex].pattern);
        
        SmartDashboard.putData("Underglow", mUnderglowPatternSelector);
    }

    public void startUnderglow() {
        mLEDUnderglow.start();
    }

    public CommandBase startUnderglowCommand() {
        return runOnce(this::startUnderglow);
    }

    public void stopUnderglow() {
        mLEDUnderglow.stop();
    }

    public CommandBase stopUnderglowCommand() {
        return runOnce(this::stopUnderglow);
    }

    public void setUnderglowPattern(CustomLEDPattern pattern) {
        pattern.setLEDs(mLEDUnderglowBuffer);
    }

    public CommandBase setUnderglowPatternCommand(CustomLEDPattern pattern) {
        return runOnce(() -> setUnderglowPattern(pattern));
    }

    @Override
    public void periodic() {
        if (mCurrentUnderglowPattern.isAnimated() || mCurrentUnderglowPattern instanceof MatchAlliancePattern) {
            mCurrentUnderglowPattern.setLEDs(mLEDUnderglowBuffer);
        }
        
        final var selected = mUnderglowPatternSelector.getSelected();
        if (selected != mCurrentUnderglowPattern) {
            mCurrentUnderglowPattern = selected;
            setUnderglowPattern(mCurrentUnderglowPattern);
        }

        mLEDUnderglow.setData(mLEDUnderglowBuffer);
    }
}