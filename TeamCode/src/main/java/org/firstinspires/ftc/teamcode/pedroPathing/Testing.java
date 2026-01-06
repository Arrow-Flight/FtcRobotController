package org.firstinspires.ftc.teamcode.pedroPathing;


import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Blue.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Shoot;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Testing extends OpMode {
boolean done = false;
    public static Pose current =  new Pose(22.8, 128, Math.toRadians(-45));

    public void init() {
        AutoConstants.init(hardwareMap);
    }

    public void loop() {
        follower.update();

        if (!done) {
            Shoot(current, shootPose);
            done = true;
        }
    }
}
