package com.spartronics4915.frc2023.bling;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class AlternatingColorPattern implements CustomLEDPattern{
	private Color[] mColors;

	private int mOffset;
	private double t;
	private final int mCogSize = 60;

	private final double kLEDCycleTime = 0.02;

	public AlternatingColorPattern(Color... colors) {
		mColors = colors;
		t = 0;
	}

	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {	
		if (mOffset == mCogSize * mColors.length) {
			mOffset = 0;
		}
		if (t % kLEDCycleTime <= 0.02) {
			mOffset++;
		};
		
		for (int index = 0; index < buffer.getLength(); index++) {
			for (int i = 0; i < mColors.length; i++) {
				if ((index + mOffset) % (mCogSize * mColors.length) / mCogSize == i) {
					buffer.setLED(index, mColors[i]);
				}
			}
		}	
		t += 1. / 50.;
	}

	@Override
	public boolean isAnimated() {
		return true;
	}
}