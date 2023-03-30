package com.spartronics4915.frc2023.bling;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class ChaosPattern implements CustomLEDPattern {
	private boolean mFirstTime = true;

	public ChaosPattern() {
		super();

	}

	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {
		if (mFirstTime){
			for (int index = 0; index < buffer.getLength(); index++) {
				buffer.setLED(index,new Color(Math.random(),Math.random(),Math.random()));
			}
			mFirstTime = false;
		}
		for (int index = 0; index < buffer.getLength(); index++) {
			buffer.setLED(index,randomColorShift(buffer.getLED(index)));
		}

	}
	
	private Color randomColorShift(Color aColor) {
		return new Color(randomShift(aColor.red),randomShift(aColor.green),randomShift(aColor.blue));
	}

	private double randomShift(double value) {
		double sign = Math.random() >= 0.5 ? 1.0 : -1.0;
		double amount = Math.random() / 10;
		return MathUtil.clamp(value + sign * amount, 0, 1);
	}

	public boolean isAnimated() {
		return true;
	}
}