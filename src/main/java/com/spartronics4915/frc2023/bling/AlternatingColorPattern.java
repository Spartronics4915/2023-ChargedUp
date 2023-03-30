package com.spartronics4915.frc2023.bling;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class AlternatingColorPattern implements CustomLEDPattern{
	private Color[] m_colors;

	public AlternatingColorPattern(Color... colors) {
		super();
		this.m_colors = colors;
	}

	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {	
		for (int index = 0; index < buffer.getLength(); index++){
			buffer.setLED(index, m_colors[index % m_colors.length]);
		}	
	}
}