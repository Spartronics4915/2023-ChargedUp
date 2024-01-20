package com.spartronics4915.frc2023.bling;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class CustomLED implements CustomLEDPattern {
    private AddressableLED mLED;
	private AddressableLEDBuffer mBuffer;
	private CustomLEDPattern mPattern;
	private Timer timer = new Timer();
	private TimerTask task;
	private int mAnimationDelay = 50;

	public CustomLED(int pwmPort, int length) {
		super();
		mLED = new AddressableLED(pwmPort);
		mBuffer = new AddressableLEDBuffer(length);
		mLED.setLength(length);
		mLED.setData(mBuffer);
		mLED.start();
	}

	public CustomLED(int pwmPort, int length, int animationSpeed) {
		this(pwmPort, length);
		this.mAnimationDelay = animationSpeed;
	}

	public AddressableLED getLED() {
		return mLED;
	}

	public void setLEDs(AddressableLEDBuffer buffer) {
		this.mBuffer = buffer;
	}

	public AddressableLEDBuffer getBuffer() {
		return mBuffer;
	}

	public void setBuffer(AddressableLEDBuffer buffer) {
		this.mBuffer = buffer;
	}

	public void setPattern(CustomLEDPattern pattern) {
		if (pattern != mPattern) {
			mPattern = pattern;
			if (task != null) {
				task.cancel();
				task = null;
			}
			if (pattern.isAnimated()) {
				task = new TimerTask() {
					public void run() {
						update();
					}
				};
				timer.scheduleAtFixedRate(task, 20, mAnimationDelay); 
			}
			update();
		}
	}

	public void update() {
		mPattern.setLEDs(getBuffer());
		getLED().setData(getBuffer());
	}
}
