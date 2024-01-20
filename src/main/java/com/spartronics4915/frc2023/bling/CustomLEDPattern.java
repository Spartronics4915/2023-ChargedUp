package com.spartronics4915.frc2023.bling;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface CustomLEDPattern {
    // public void setLEDs(AddressableLEDBuffer buffer);
    public default void setLEDs(AddressableLEDBuffer buffer) {
        setLEDs(buffer, 0);
    }
    public default void setLEDs(AddressableLEDBuffer buffer, int startIndex) {
        setLEDs(buffer, startIndex, buffer.getLength());
    }
    public default void setLEDs(AddressableLEDBuffer buffer, int startIndex, int endIndex) {
        for (int i = startIndex; i < buffer.getLength() && i < endIndex; i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
    }
    public default void setLEDSection(AddressableLEDBuffer buffer, Pair<Integer, Integer> section) {
        setLEDs(buffer, section.getFirst(), section.getSecond());
    }
    public default boolean isAnimated() {
        return false;
    }
}
