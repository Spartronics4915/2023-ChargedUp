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
	public void setLEDs(AddressableLEDBuffer buffer, int startIndex, int endIndex) {
		if (mOffset == mCogSize * mColors.length) {
			mOffset = 0;
		}
		if (t % kLEDCycleTime <= 0.02) {
			mOffset++;
		};
		
		for (int i = startIndex; i < buffer.getLength() && i < endIndex; i++) {
			for (int j = 0; j < mColors.length; j++) {
				if ((i + mOffset) % (mCogSize * mColors.length) / mCogSize == j) {
					buffer.setLED(i, mColors[j]);
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