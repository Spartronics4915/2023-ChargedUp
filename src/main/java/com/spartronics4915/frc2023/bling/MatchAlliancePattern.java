package com.spartronics4915.frc2023.bling;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

public class MatchAlliancePattern implements CustomLEDPattern {
    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        final Alliance alliance = DriverStation.getAlliance();
        final Color color;
        switch (alliance) {
            case Blue: {
                color = new Color(0, 0, 255);
                break;
            }
            case Red: {
                color = new Color(255, 0, 0);
                break;
            }
            default: {
                color = new Color(0, 0, 0);
                break;
            }
        }
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
    }
}
