package com.spartronics4915.frc2023.bling;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class BlinkingPattern implements CustomLEDPattern {
	private CustomLEDPattern mOnPattern;
	private CustomLEDPattern mOffPattern;
	private double mInterval;
	private boolean on = true;
	private double lastChange;

	/**
	 * 
	 * @param onColor color for when the blink is on.
	 * @param inteval time in seconds between changes.
	 */
	public BlinkingPattern(Color onColor, double interval) {
		super();
		mOnPattern = new SolidColorPattern(onColor);
		mOffPattern = new SolidColorPattern(Color.kBlack);
		mInterval = interval;
	}
	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {
		double timestamp = Timer.getFPGATimestamp();
		if (timestamp- lastChange > mInterval) {
			on = !on;
			lastChange = timestamp;
		}
		if (on) {
			mOnPattern.setLEDs(buffer);
		} else {
			mOffPattern.setLEDs(buffer);
		}
	}
	public boolean isAnimated() {
		return true;
	}
}