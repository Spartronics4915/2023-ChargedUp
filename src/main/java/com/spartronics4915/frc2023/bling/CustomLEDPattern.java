package com.spartronics4915.frc2023.bling;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface CustomLEDPattern {
    public void setLEDs(AddressableLEDBuffer buffer);
    default boolean isAnimated() {
        return false;
    }
}
