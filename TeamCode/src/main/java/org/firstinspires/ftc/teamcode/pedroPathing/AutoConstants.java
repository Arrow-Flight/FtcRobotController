package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Blue.shootPose;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class AutoConstants {
    // ===Limelight===
    public static Limelight3A limelight;

    // ===Servos===
    public static Servo servoCamera;

    // ===Motors===
    public static DcMotor intake;
    public static DcMotor upper;
    public static DcMotorEx shooterLeft;
    public static DcMotorEx shooterRight;

    // ===Paths===
    public static Path spikeMarkPath;
    public static Path toShoot;
    public static Path goToEnd;

    // ===Misc.===
    public static Follower follower;
    public static Timer pathTimer;
    public static Timer timeout;
    public static int pathState;
    public static Pose currentPose;
    public static int subState;
    public static boolean timeoutTriggered = false;
    public static double xError = 0;
    public static double yError = 0;

    public static class Blue {
        // ===Poses===
        public static Pose startingPose = new Pose(23, 128, Math.toRadians(-36));
        public static Pose shootPose = new Pose(56, 92, Math.toRadians(-36));
        public static Pose firstSpikeInitial = new Pose(50, 85, Math.toRadians(180));
        public static Pose firstSpikeFinal = new Pose(20,85, Math.toRadians(180));
        public static Pose secondSpikeInitial = new Pose(50,62, Math.toRadians(180));
        public static Pose secondSpikeFinal = new Pose(15,62, Math.toRadians(180));
        public static Pose thirdSpikeInitial = new Pose(50,40, Math.toRadians(180));
        public static Pose thirdSpikeFinal = new Pose(15,40, Math.toRadians(180));
        public static Pose endPose = new Pose(35,75, Math.toRadians(0));

        public static int pipeline = 7;
    }
    public static class Red {
        // ===Poses===
        public static Pose startingPose = new Pose(121.2, 128, Math.toRadians(-135));
        public static Pose shootPose = new Pose(92, 92, Math.toRadians(-138));
        public static Pose firstSpikeInitial = new Pose(95, 85, Math.toRadians(5));
        public static Pose firstSpikeFinal = new Pose(124,85, Math.toRadians(5));
        public static Pose secondSpikeInitial = new Pose(95,62, Math.toRadians(5));
        public static Pose secondSpikeFinal = new Pose(134,62, Math.toRadians(5));
        public static Pose thirdSpikeInitial = new Pose(95,40, Math.toRadians(5));
        public static Pose thirdSpikeFinal = new Pose(134,40, Math.toRadians(5));
        public static Pose endPose = new Pose(115,75, Math.toRadians(180));

        public static int pipeline = 8;
    }

    public static void Shoot(Telemetry telemetry) {
           telemetry.addData("SHOT?", "YES!");
    }

    public static void collectSpike(int state, Pose start, Pose end) {
        if (subState == 1 && !follower.isBusy()) {

            spikeMarkPath = new Path(new BezierLine(currentPose, start));
            spikeMarkPath.setLinearHeadingInterpolation(currentPose.getHeading(), start.getHeading());

            follower.followPath(spikeMarkPath);
            subState = 2;

        } else if (subState == 2 && !follower.isBusy()) {
            intake.setPower(1);

            spikeMarkPath = new Path(new BezierLine(start, end));
            spikeMarkPath.setLinearHeadingInterpolation(start.getHeading(),end.getHeading());

            follower.followPath(spikeMarkPath);
            subState = 3;
            pathState = state;
        }
    }

    public static void goToShoot(){
        toShoot = new Path(new BezierLine(currentPose, shootPose));
        toShoot.setLinearHeadingInterpolation(currentPose.getHeading(), shootPose.getHeading());

        follower.followPath(toShoot);
    }

    public static Pose getStartingError() {
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            double xInches = botPose.getPosition().x * 39.37;
            double yInches = botPose.getPosition().y * 39.37;

            double targetX = xInches + 72;
            double targetY = -yInches + 72;

            double xError = targetX - currentPose.getX();
            double yError = targetY - currentPose.getY();

            return new Pose(xError,yError);
        } else return new Pose(0,0);
    }

    public static Pose getCorrectedPose() {
        Pose current = follower.getPose();
        return new Pose(current.getX() + xError, current.getY() + yError, current.getHeading());
    }
}

