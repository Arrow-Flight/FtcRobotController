package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class IfPractice extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        boolean aButton = gamepad1.a; // press TRUE, depress False
        double leftY = gamepad1.left_stick_y;

        if (aButton) {
            telemetry.addData("A Button", "Pressed!");
        }
            else { //otherwise
                telemetry.addData("A Button", "NOT Pressed");
        }
            telemetry.addData("A Button State", aButton);


            if (leftY < 0 && aButton) { // Outputs Y value when a is pressed
                telemetry.addData("Left Stick", "is Negative");
            } else if (leftY > 0 && aButton) {
                telemetry.addData("Left Stick","is Positive");

            } else {
                telemetry.addData("Left Stick", "is Zero");
            }
    }
}
