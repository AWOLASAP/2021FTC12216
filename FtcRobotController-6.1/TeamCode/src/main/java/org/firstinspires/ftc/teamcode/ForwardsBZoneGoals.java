// Copyright (c) 2017 FIRST. All rights reserved.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Forwards B Zone Goals", group="Autonomous")
public class ForwardsBZoneGoals extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_right = null;
    private DcMotor front_left = null;
    private DcMotor back_right = null;
    private DcMotor back_left = null;

    private DcMotor wobble_arm = null;
    private DcMotor launcher = null;
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
        launcher = hardwareMap.get(DcMotor.class, "launcher");
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

        while (opModeIsActive()) {
            // Drop the wobble claw
            wobbleClaw(1);

            // Pause for the servo
            pause(1000);

            // Move backwards into the B zone
            frontBack(1, 2900);

            // Stop before moving over the line
            stopRobot();

            // Move forward over the line
            frontBack(-1, 1300);

            stopRobot();

            pause(100);

            // Twist a bit
            twist(1, 100);

            // Launch a ring
            launcher.setPower(-0.7);
            pause(500);
            fire_servo.setPosition(0.05);
            pause(500);
            fire_servo.setPosition(0.45);

            pause(1000);


            // Launch a ring
            launcher.setPower(-.7);
            pause(500);
            fire_servo.setPosition(0.05);
            pause(500);
            fire_servo.setPosition(0.45);

            pause(1000);


            // Launch a ring
            launcher.setPower(-.7);
            pause(500);
            fire_servo.setPosition(0.05);
            pause(500);
            fire_servo.setPosition(0.45);

            pause(1000);

            stopRobot();

            break;
        }
    }

}
