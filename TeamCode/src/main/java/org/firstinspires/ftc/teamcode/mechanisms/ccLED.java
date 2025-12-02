package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class ccLED {

    private LED redLed;
    private LED greenLed;

    public void init(HardwareMap map, String ledName) {
        redLed = map.get(LED.class, ledName + "_red");
        greenLed = map.get(LED.class, ledName + "_green");
    }

    public void setRedLed() {
        redLed.on();
        greenLed.off();
    }

    public void setGreenLed() {
        greenLed.on();
        redLed.off();
    }

    public void setLedOff() {
        greenLed.off();
        redLed.off();
    }
}
