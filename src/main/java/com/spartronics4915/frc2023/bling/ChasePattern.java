package com.spartronics4915.frc2023.bling;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class ChasePattern implements CustomLEDPattern {
	private Color[] mColors;
	private int mSegmentWidth;
	private int mOffset;
	public ChasePattern(Color[] colors, int segmentWidth){
		super();
		mColors = colors;
		mSegmentWidth = segmentWidth;
	}

	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {
		int numberOfColors = mColors.length;
		int effectiveIndex;
		int colorIndex;
		int bufferLength = buffer.getLength();
		for (int index = 0; index < bufferLength; index++){
			effectiveIndex = (index + mOffset) % bufferLength;
			colorIndex =( index /mSegmentWidth )% numberOfColors;
			buffer.setLED(effectiveIndex, mColors[colorIndex]);
		}

		mOffset =(mOffset+1) %bufferLength;
	}
	public boolean isAnimated(){
		return true;
	}
}
