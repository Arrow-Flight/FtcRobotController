package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.PrimitiveIterator;

public class TestBench {
    private DigitalChannel touchSensor;
    private DcMotor motor;
    private double ticksPerRev; //revolutions

    public void init(HardwareMap haMap) {
        //touch sensor
        touchSensor = haMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        //DC motor
        motor =haMap.get(DcMotor.class, "leftBack");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRev = motor.getMotorType().getTicksPerRev();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Activates Zero Power Breaking
        motor.setDirection(DcMotorSimple.Direction.REVERSE); // Reverses direction\
    }

    // Touch Sensor
    public boolean getTouchSensorState() {
        return !touchSensor.getState(); // touch sensors return true when up by default, needs !(not) to flip
    }

    public boolean isTouchSensorReleased() { // Touch Sensor Setter Method
        return !touchSensor.getState();
    }

    //DC Motor
    public void setMotorSpeed(double speed) { // DC Motor Setter Method
        // accepts values from -1.0 to 1.0
        motor.setPower(speed);
    }
    public double getMotorRevs() {
        return motor.getCurrentPosition() / ticksPerRev; // normalizes ticks to revolutions
    }
}
