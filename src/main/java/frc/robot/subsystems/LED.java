package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class LED extends SubsystemBase {
  private final AddressableLED led = new AddressableLED(Constants.LEDs.LED_DIO_PORT);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LEDs.LED_LENGTH);
  private int rainbowFirstPixelHue;

  public LED() {
    this.led.setLength(ledBuffer.getLength());
    this.led.setData(ledBuffer);
    this.led.start();
  }

  @Override
  public void periodic() {

  }

  public void setLEDs() {
    this.ledBuffer.setRGB(10, 255, 0, 0);
    this.led.setData(ledBuffer);
    this.led.start();
  }

  // this is for fun if i'm bored
  public void demiFlag() {
  }

  public void agenFlag() {
  }

  public void rainbow() {
    // For every pixel
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Set the value
      this.ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    this.rainbowFirstPixelHue += 3;
    // Check bounds
    this.rainbowFirstPixelHue %= 180;
    this.led.setData(ledBuffer);
  }

}