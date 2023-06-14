// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Util.UnitsConvert;


public class WS2812Driver extends SubsystemBase {
  /** Creates a new WS2812Driver. */
  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;
  int breathe = 255;
  boolean breatheReversed = false;
  int breatheH = 10;
  int blinkCount = 0;
  private int beginning;  
  private int m_rainbowFirstPixelHue;
  private int emergency_beginning;
  private int j;
   int blinkFrequency;
   int value;
   boolean ismaxbreathing;
  private int scannerPosition = 0;
  private int scannerDirection = 1;

  public WS2812Driver(int dataPort, int ledLength) {
    m_led = new AddressableLED(dataPort);
    m_ledBuffer = new AddressableLEDBuffer(ledLength);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();
  }

  @Override
  public void periodic() {
  }

  public static void setColor(int r, int g, int b) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
}

  public void turnOff() {
    setColor(0, 0, 0);
    m_led.setData(m_ledBuffer);
}

public void blink(int r, int g, int b,int blinkFrequency) {
  if (blinkFrequency != 0) {
      if (j % blinkFrequency == 0) {
          setColor(0, 0, 0);
          j = 1;
      } else if (j % (blinkFrequency / 2) == 0) {
        setColor(r, g, b);
      }
      j++;
  } else {
    setColor(r, g, b);
  }
}

  public int[] shiftArray(int[] array){
    int last = array[array.length-1];
    for(int i = array.length-1; i > 0; i--){
      array[i] = array[i-1];
    }
    array[0] = last;
    return array;
  }
  public void emergency(int errorLength){
    for(var i = 0; i < m_ledBuffer.getLength();i++){
      if(i>=emergency_beginning&&i<=emergency_beginning+errorLength){
        m_ledBuffer.setLED(i, new Color(255,0,0));
      }
      else m_ledBuffer.setRGB(i, 0, 0, 0);
    }
    m_led.setData(m_ledBuffer);
    emergency_beginning++;
    emergency_beginning %= 44; 
  }

  public void sliding(int r, int g, int b){
    for(var i = 0; i < m_ledBuffer.getLength();i++){
      if(i>=beginning&&i<=beginning+1){
        m_ledBuffer.setRGB(i, 0, 0, 0);
      }
      else m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
    beginning++;
    beginning %= 11; 
  }

  public void larson(int r, int g, int b,int scannerlenght){
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
  }
  m_ledBuffer.setRGB(scannerPosition, r,g, b);

  scannerPosition += scannerDirection;

  if (scannerPosition >= m_ledBuffer.getLength() - scannerlenght || scannerPosition <= 0) {
      scannerDirection *= -1;
  }

  // Show the updated buffer on the LED strip
  m_led.setData(m_ledBuffer);
  }

  public void toggleRGB(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
        m_ledBuffer.setHSV(i, hue, 255, 128);
      }
      m_rainbowFirstPixelHue += 3;
      m_rainbowFirstPixelHue %= 180;
  }
  // public void breathe(int r, int g, int b){
  //   for (var i = 0; i < m_ledBuffer.getLength(); i++) {
  //     final var hue = UnitsConvert.rgbToHue(r, g, b);
  //       m_ledBuffer.setHSV(i, hue, 255, value*3);
  //     }
  //     m_led.setData(m_ledBuffer);
  //     if (ismaxbreathing){
  //       value--;
  //     }else{
  //       value++;
  //     }
  //     if (value==83){
  //       ismaxbreathing=true;
  //     }else if(value==0){
  //       ismaxbreathing=false;
  //     }
  // }









private void setPixelHeatColor(int pixel, int heat) {
  // Convert heat value to RGB color
  int r, g, b;
  if (heat < 85) {
      r = heat * 3;
      g = 0;
      b = 0;
  } else if (heat < 170) {
      heat -= 85;
      r = 255 - heat * 3;
      g = 0;
      b = 0;
  } else {
      heat -= 170;
      r = 0;
      g = 0;
      b = heat * 3;
  }
  m_ledBuffer.setRGB(pixel, r, g, b);
}
public void fire(){
  for (int i = 0; i < m_ledBuffer.getLength(); i++) {
    int heat = (int) (Math.random() * 255); // Generate random heat value (0-255)
    setPixelHeatColor(i, heat);
}
m_led.setData(m_ledBuffer);
}



}