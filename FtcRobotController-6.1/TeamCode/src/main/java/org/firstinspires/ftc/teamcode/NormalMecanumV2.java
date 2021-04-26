// Copyright (c) 2017 FIRST. All rights reserved.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Normal MecanumV2", group="Iterative Opmode")
public class NormalMecanumV2 extends OpMode
{
    // Declare OpMode members.
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

    private Servo Yaw = null;
    private Servo Pitch = null;

    // Pause, just pause
    public void pause(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

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

        Yaw = hardwareMap.get(Servo.class, "yaw");
        Pitch = hardwareMap.get(Servo.class, "pitch");


        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobble_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() { }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() { }

    // Launcher with encoders
    double launcherVelocity = 0;
    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        // Position variable for the wobble claw
        double wobble_claw_position = 0.0;

        // Wobble arm power, don't go overboard
        double wobblePower = 0;
        if (gamepad1.right_trigger > 0.05) {
            wobblePower = -gamepad1.right_trigger / 2;
        } else if (gamepad1.left_trigger > 0.05) {
            wobblePower = gamepad1.left_trigger / 2;
        }

        // Wobble hand servo
        if (gamepad1.a) {
            wobble_servo.setPosition(0.38);
        } else if (gamepad1.b) {
            wobble_servo.setPosition(0.0);
        }

        // Fire servo
        if (gamepad1.x) {
            fire_servo.setPosition(0.45);
        } else if (gamepad1.y) {
            fire_servo.setPosition(0.05);
        }

        // Fire servo complete
        if (gamepad1.right_bumper) {
            fire_servo.setPosition(0.05);
            pause(250);
            fire_servo.setPosition(0.45);
        }

        // Launcher motor
        /*
        if (gamepad1.dpad_right) {
            launcher.setPower(-0.725);
            //launcher.setPower(-1);
        } else if (gamepad1.start) {
            launcher.setPower(-0.675);
        } else if (gamepad1.dpad_left) {
            launcher.setPower(0);
        }
        */

        if (gamepad1.dpad_right) {
            launcherVelocity = -2000;
        } else if (gamepad1.start) {
            launcherVelocity = -1910;
        } else if (gamepad1.dpad_left) {
            launcherVelocity = 0;
        }

        launcher.setVelocity(launcherVelocity);

        // Collector motor
        if (gamepad1.dpad_up) {
            collector.setPower(-1);
        } else if (gamepad1.dpad_down) {
            collector.setPower(0);
        }

        // Drop or raise the wobble claw
        if (gamepad1.left_bumper) {
            wobble_claw_position = 0.8;
            wobble_claw.setPosition(wobble_claw_position);
        } else if (gamepad1.back) {
            wobble_claw_position = 0.4;
            wobble_claw.setPosition(wobble_claw_position);
        }

        // Gimbal
        Yaw.setPosition(0.24);
        Pitch.setPosition(0.78);


        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twsist (rotating the whole chassis).

        double frontRightPower = 0;
        double frontLeftPower = 0;
        double backRightPower = 0;
        double backLeftPower = 0;

        frontRightPower -= gamepad1.right_stick_y;
        frontLeftPower -= gamepad1.right_stick_y;
        backLeftPower -= gamepad1.right_stick_y;
        backRightPower -= gamepad1.right_stick_y;

        frontRightPower -= gamepad1.right_stick_x;
        frontLeftPower -= -gamepad1.right_stick_x;
        backLeftPower -= gamepad1.right_stick_x;
        backRightPower -= -gamepad1.right_stick_x;

        frontRightPower -= -gamepad1.left_stick_x;
        frontLeftPower -= gamepad1.left_stick_x;
        backLeftPower -= gamepad1.left_stick_x;
        backRightPower -= -gamepad1.left_stick_x;

        front_right.setPower(frontRightPower);
        front_left.setPower(frontLeftPower);
        back_right.setPower(backRightPower);
        back_left.setPower(backLeftPower);
        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */


        // apply power to wobble arm
        wobble_arm.setPower(wobblePower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Thingy: ", "Launcher Encoder" + launcher.getCurrentPosition());
        // Show wheel power
        telemetry.addData("Motor", "roll (%.2f), pitch (%.2f)", ((gamepad2.left_stick_x + 1 )/2), ((gamepad2.right_stick_y + 1) / 2));

    }

}
