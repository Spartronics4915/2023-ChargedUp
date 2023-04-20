package com.spartronics4915.frc2023.bling;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class SolidColorPattern implements CustomLEDPattern{
    private Color mColor;

    public SolidColorPattern(Color aColor) {
        super();
        this.mColor = aColor;
    }

    @Override
    public void setLEDs(AddressableLEDBuffer buffer, int startIndex, int endIndex) {
        for (int i = 0; i < buffer.getLength() && i < endIndex; i++) {
            buffer.setLED(i, mColor);
        }
    }
}
