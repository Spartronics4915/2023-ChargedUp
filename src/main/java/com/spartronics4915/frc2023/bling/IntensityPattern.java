package com.spartronics4915.frc2023.bling;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class IntensityPattern  implements CustomLEDPattern {
	private Color mHighColor;
	private Color mLowColor;
	private double mIntensity;

	private double t;

	/**
	 * 
	 * @param highColor Brightest color
	 * @param intensity 0..1 with 1 being the color and 0 being black 
	 */
	public IntensityPattern(Color highColor) {
		this(Color.kBlack,highColor);
	}

	public IntensityPattern(Color lowColor, Color highColor) {
		this.mHighColor = highColor;
		this.mLowColor = lowColor;
		mIntensity = 0;
	}

	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {
		double red = MathUtil.interpolate(mLowColor.red, mHighColor.red, mIntensity);
		double green = MathUtil.interpolate(mLowColor.green, mHighColor.green, mIntensity);
		double blue = MathUtil.interpolate(mLowColor.blue, mHighColor.blue, mIntensity);
		for (int index = 0; index < buffer.getLength(); index++){
			buffer.setLED(index, new Color(red,green,blue));
		}
		t += 1. / 50.;
		setIntensity(Math.cos(2 * Math.PI * t));
	}

	@Override
	public boolean isAnimated() {
		return true;
	}

	public void setIntensity(double intensity){
		mIntensity = intensity;
	}
}
