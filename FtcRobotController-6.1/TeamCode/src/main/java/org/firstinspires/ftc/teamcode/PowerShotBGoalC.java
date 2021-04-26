// Copyright (c) 2017 FIRST. All rights reserved.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Powershot B Goal C", group="Autonomous")
public class PowerShotBGoalC extends LinearOpMode
{
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_right = null;
    private DcMotor front_left = null;
    private DcMotor back_right = null;
    private DcMotor back_left = null;

    private DcMotor wobble_arm = null;
    private DcMotorEx launcher = null;
    private DcMotor collector = null;

    private Servo wobble_servo = null;
    private Servo fire_servo = null;
    private Servo wobble_claw = null;

    //// User Functions
    // Stop the robot
    public void stopRobot() {
        // NOT SO MUCH FULL POWAH!
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    // Move Forward or Backwards
    public void frontBack(double powah, int time) {
        // USER DEFINED POWAH!
        front_left.setPower(powah);
        front_right.setPower(powah);
        back_left.setPower(powah);
        back_right.setPower(powah);

        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        // NOT SO MUCH FULL POWAH!
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    // Twist a bit
    public void twist(int dir, int time) {
        if (dir > 0) {
            // USER DEFINED POWAH!
            front_left.setPower(-1);
            front_right.setPower(1);
            back_left.setPower(-1);
            back_right.setPower(1);
        } else {
            // USER DEFINED POWAH!
            front_left.setPower(1);
            front_right.setPower(-1);
            back_left.setPower(1);
            back_right.setPower(-1);
        }

        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        // NOT SO MUCH FULL POWAH!
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    // Slowly twist a bit
    public void slowTwist(int dir, int time) {
        if (dir > 0) {
            // USER DEFINED POWAH!
            front_left.setPower(-0.5);
            front_right.setPower(0.5);
            back_left.setPower(-0.5);
            back_right.setPower(0.5);
        } else {
            // USER DEFINED POWAH!
            front_left.setPower(0.5);
            front_right.setPower(-0.5);
            back_left.setPower(0.5);
            back_right.setPower(-0.5);
        }

        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        // NOT SO MUCH FULL POWAH!
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    // Drop or lower robot claw
    public void wobbleClaw(int down) {
        // Drop or raise the wobble claw
        if (down > 0) {
            wobble_claw.setPosition(0.8);
        } else {
            wobble_claw.setPosition(0.3);
        }
    }

    // Pause, just pause
    public void pause(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    //// End of User Functions


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables.

        // Initialize the hardware variables.
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");

        wobble_arm = hardwareMap.get(DcMotor.class, "wobble_arm");
        launcher  = hardwareMap.get(DcMotorEx.class, "launcher");
        collector = hardwareMap.get(DcMotor.class, "collector");

        wobble_servo = hardwareMap.get(Servo.class, "wobble_servo");
        fire_servo = hardwareMap.get(Servo.class, "fire_servo");
        wobble_claw = hardwareMap.get(Servo.class, "wobble_claw");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        runtime.reset();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // Launcher with encoders
        double launcherVelocity = 0;
        while (opModeIsActive()) {
            // Drop the wobble claw
            wobbleClaw(1);

            // Pause for the servo
            pause(1000);

            // Move Fowards to launch line
            frontBack(1, 1800);

            pause(100);

            // Launch a ring
            launcherVelocity = -1910;
            launcher.setVelocity(launcherVelocity);
            pause(750);
            fire_servo.setPosition(0.05);
            pause(500);
            fire_servo.setPosition(0.45);

            pause(1000);

            // Twist a bit
            twist(1, 70);

            // Launch a ring
            pause(500);
            fire_servo.setPosition(0.05);
            pause(500);
            fire_servo.setPosition(0.45);

            launcherVelocity = -1920;
            launcher.setVelocity(launcherVelocity);
            pause(1000);

            // Twist a bit
            twist(1, 70);

            // Launch a ring
            pause(500);
            fire_servo.setPosition(0.05);
            pause(500);
            fire_servo.setPosition(0.45);

            pause(1000);

            // Twist to B zone
            slowTwist(-1, 900);

            // Move to be zone
            frontBack(1, 2400);

            // Move back over the line
            frontBack(-1, 1800);

            stopRobot();

            break;
        }
    }

}
