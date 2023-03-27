package com.spartronics4915.frc2023.bling;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class RadarPattern implements CustomLEDPattern{
    private Color mEyeColor;
    private Color mBackgroundColor;
    private int mLength;
    private int mEyePosition = 0;
    private int mScanDirection = 1;

    public RadarPattern(Color eyeColor, int length){
        this(eyeColor, Color.kBlack, length);
    }

    public RadarPattern(Color eyeColor, Color backgroundColor, int length){
        super();
        this.mEyeColor = eyeColor;
        this.mBackgroundColor = backgroundColor;
        this.mLength = length;
    }

    @Override
	public void setLEDs(AddressableLEDBuffer buffer) {
		int bufferLength = buffer.getLength();
		double intensity;
		double red;
		double green;
		double blue;
		double distanceFromEye;

		for (int index = 0; index < bufferLength; index++) {
			distanceFromEye = MathUtil.clamp( Math.abs(mEyePosition - index),0,mLength);
			intensity = 1 - distanceFromEye/mLength;
			red = MathUtil.interpolate(mBackgroundColor.red, mEyeColor.red, intensity);
			green = MathUtil.interpolate(mBackgroundColor.green, mEyeColor.green, intensity);
			blue = MathUtil.interpolate(mBackgroundColor.blue, mEyeColor.blue, intensity);

			buffer.setLED(index, new Color(red, green, blue));
		}

		if (mEyePosition == 0) {
			mScanDirection = 1;
		} else if (mEyePosition == bufferLength - 1) {
			mScanDirection = -1;
		}

		mEyePosition += mScanDirection;
	}

	@Override
	public boolean isAnimated() {
		return true;
	}
}
