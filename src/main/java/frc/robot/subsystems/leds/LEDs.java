// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDs extends SubsystemBase {
  // TODO: Add Constants file
  
  // PWM port
  private static final int kLedPort = 5;
  // LED strip length (allegedly)
  private static final int kLedLength = 30;

  // The LED strip itself
  private final AddressableLED m_led;
  // Patterns go here first
  private final AddressableLEDBuffer m_ledBuffer;

  public LEDs() {
    m_led = new AddressableLED(kLedPort);
    m_ledBuffer = new AddressableLEDBuffer(kLedLength);
    m_led.setLength(kLedLength);
    m_led.start();

  }

  @Override
  public void periodic() {
    // Send the data to the led strip
    m_led.setData(m_ledBuffer);
  }
}
