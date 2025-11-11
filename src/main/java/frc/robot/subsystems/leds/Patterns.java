// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

/** All the patterns for the LEDs (some are currently untested) */
public class Patterns {
    /** Solid colors */
    public static class Solids {
        public static final LEDPattern ORANGE = LEDPattern.solid(Color.kOrangeRed);
        public static final LEDPattern WHITE = LEDPattern.solid(Color.kWhite);
        public static final LEDPattern BLUE = LEDPattern.solid(Color.kBlue);
        public static final LEDPattern GREEN = LEDPattern.solid(Color.kLime);
        public static final LEDPattern YELLOW = LEDPattern.solid(Color.kYellow);
        public static final LEDPattern PURPLE = LEDPattern.solid(Color.kPurple);
        public static final LEDPattern RED = LEDPattern.solid(Color.kRed);
        public static final LEDPattern CYAN = LEDPattern.solid(Color.kCyan);
        public static final LEDPattern OFF = LEDPattern.solid(Color.kBlack);
    }
}
