package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBench {
    private DigitalChannel touchSensor;

    public void init(HardwareMap haMap) {
        touchSensor = haMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getTouchSensorState() {
        return !touchSensor.getState(); // touch sensors return true when up by default, needs !(not) to flip
    }

    public boolean isTouchSensorReleased() {
        return !touchSensor.getState();
    }
}
