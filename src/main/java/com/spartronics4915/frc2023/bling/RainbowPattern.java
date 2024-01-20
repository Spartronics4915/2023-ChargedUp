package com.spartronics4915.frc2023.bling;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RainbowPattern implements CustomLEDPattern{
	private int mFirstHue = 0;
	public RainbowPattern() {}

	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {
		int currentHue;
		for (int index = 0; index < buffer.getLength(); index++){
			currentHue = (mFirstHue + (index * 180 / buffer.getLength())) % 180;
			buffer.setHSV(index, currentHue, 255, 128);
		}

		mFirstHue = (mFirstHue + 3) % 180;
	}
	public boolean isAnimated() {
		return true;
	}
}