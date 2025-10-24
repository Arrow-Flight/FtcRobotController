package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.TestBench;

@Disabled
@TeleOp
public class TouchSensorPractice extends OpMode {
    TestBench bench = new TestBench();

    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Touch Sensor State", bench.getTouchSensorState());
        if (bench.isTouchSensorReleased()) {
            telemetry.addData("Touch Sensor Released?", "Pressed!");
        }
        else {
            telemetry.addData("Touch Sensor Released?", "Not Pressed");
        }
    }
}
