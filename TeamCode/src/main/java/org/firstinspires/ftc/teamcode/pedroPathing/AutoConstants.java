package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class AutoConstants {
    // ===Limelight===
    public static Limelight3A limelight;

    // ===PIDF==
    public static PIDFController pidfController;

    // ===Servos===
    public static Servo servoCamera;

    // ===Motors===
    public static DcMotorEx intake;
    public static DcMotor upper;
    public static DcMotor shooterLeft;
    public static DcMotor shooterRight;

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
    public static int shots;
    public static int shootState;
    public static boolean spunUp;
    public static double vel;

    public static class Blue {
        // ===Poses===
        public static Pose startingPose = new Pose(23, 128, Math.toRadians(-36));
        public static Pose shootPose = new Pose(45, 102, Math.toRadians(-36));
        public static Pose firstSpikeInitial = new Pose(50, 85, Math.toRadians(180));
        public static Pose firstSpikeFinal = new Pose(20,85, Math.toRadians(180));
        public static Pose secondSpikeInitial = new Pose(50,62, Math.toRadians(180));
        public static Pose secondSpikeFinal = new Pose(20,62, Math.toRadians(180));
        public static Pose thirdSpikeInitial = new Pose(50,40, Math.toRadians(180));
        public static Pose thirdSpikeFinal = new Pose(20,40, Math.toRadians(180));
        public static Pose endPose = new Pose(35,75, Math.toRadians(0));

        public static int pipeline = 7;

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
    }
    public static class Red {
        // ===Poses===
        public static Pose startingPose = new Pose(121, 128, Math.toRadians(-144));
        public static Pose shootPose = new Pose(99, 116, Math.toRadians(-144));
        public static Pose firstSpikeInitial = new Pose(94, 90, Math.toRadians(0));
        public static Pose firstSpikeFinal = new Pose(124,90, Math.toRadians(0));
        public static Pose secondSpikeInitial = new Pose(94,67, Math.toRadians(0));
        public static Pose secondSpikeFinal = new Pose(124,67, Math.toRadians(0));
        public static Pose thirdSpikeInitial = new Pose(94,45, Math.toRadians(0));
        public static Pose thirdSpikeFinal = new Pose(124,45, Math.toRadians(0));
        public static Pose endPose = new Pose(105,75, Math.toRadians(180));

        public static int pipeline = 8;

        public static Pose getStartingError() {
            LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose();
                double xInches = botPose.getPosition().x * 39.37;
                double yInches = botPose.getPosition().y * 39.37;

                double targetX = -xInches + 72;
                double targetY = yInches + 72;

                double xError = targetX - currentPose.getX();
                double yError = targetY - currentPose.getY();

                return new Pose(xError,yError);
            } else return new Pose(0,0);
        }
    }

    public static void Shoot(int state) {
        if (shootState == 1) {
            pidfController.updatePosition(vel);
            pidfController.updateFeedForwardInput(0.76);

            double prePower = pidfController.run();
            double power = Math.max(-1.0, Math.min(1.0, prePower));

            shooterLeft.setPower(power);
            shooterRight.setPower(power);
            shots = 0;
            if (vel > 2800) {
                intake.setPower(1);
                upper.setPower(0.5);
                pathTimer.resetTimer();
                shootState = 2;
            }
        }
        else if (shootState == 2 && pathTimer.getElapsedTimeSeconds() >= 1) {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
            intake.setPower(0);
            upper.setPower(0);

            pathState = state;
        }
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

    public static void goToShoot(Pose shootAt){
        toShoot = new Path(new BezierLine(currentPose, shootAt));
        toShoot.setLinearHeadingInterpolation(currentPose.getHeading(), shootAt.getHeading());

        follower.followPath(toShoot);
    }

    public static Pose getCorrectedPose() {
        Pose current = follower.getPose();
        return new Pose(current.getX() + xError, current.getY() + yError, current.getHeading());
    }

    public static long lastPosition = 0;
    public static long lastTime = 0;
    public static final int VELOCITY_BUFFER_SIZE = 5;
    public static double[] velocityBuffer = new double[VELOCITY_BUFFER_SIZE];
    public static int bufferIndex = 0;
    public static int bufferFilled = 0;

    public static double getVelocity() {
        long currentPosition = intake.getCurrentPosition();
        long currentTime = System.nanoTime();

        if (lastTime == 0) {
            lastPosition = currentPosition;
            lastTime = currentTime;
            return 0;
        }

        long delta = currentPosition - lastPosition;
        double deltaTime = (currentTime - lastTime) / 1e9;
        double instantVelocity = (delta / deltaTime / 8192.0) * 60.0;

        // Spike rejection — ignore readings that are too far from current average
        double currentAverage = 0;
        if (bufferFilled > 0) {
            double sum = 0;
            for (int i = 0; i < bufferFilled; i++) sum += velocityBuffer[i];
            currentAverage = sum / bufferFilled;
        }

        if (bufferFilled == 0 || Math.abs(instantVelocity - currentAverage) < 300) {
            velocityBuffer[bufferIndex] = instantVelocity;
            bufferIndex = (bufferIndex + 1) % VELOCITY_BUFFER_SIZE;
            if (bufferFilled < VELOCITY_BUFFER_SIZE) bufferFilled++;
        }

        // Weighted average — newer samples weighted higher
        double weightedSum = 0;
        double weightSum = 0;
        for (int i = 0; i < bufferFilled; i++) {
            int age = (bufferIndex - 1 - i + VELOCITY_BUFFER_SIZE) % VELOCITY_BUFFER_SIZE;
            double weight = bufferFilled - age;
            weightedSum += velocityBuffer[i] * weight;
            weightSum += weight;
        }

        lastPosition = currentPosition;
        lastTime = currentTime;

        return weightSum > 0 ? weightedSum / weightSum : 0;
    }
}

