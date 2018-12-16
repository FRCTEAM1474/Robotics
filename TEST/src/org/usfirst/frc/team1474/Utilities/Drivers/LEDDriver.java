package org.usfirst.frc.team1474.Utilities.Drivers;

import org.usfirst.frc.team1474.Utilities.RGBColor;

public interface LEDDriver {
	void set(boolean on);
	void setLEDColor(RGBColor rgbColor);
}
