// Copyright (c) 2017 FIRST. All rights reserved.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Normal Mecanum", group="Iterative Opmode")
public class NormalMecanum extends OpMode
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
        if (gamepad1.dpad_right) {
            launcher.setPower(-0.725);
            //launcher.setPower(-1);
        } else if (gamepad1.dpad_left) {
            launcher.setPower(0);
        }

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
            wobble_claw_position = 0.3;
            wobble_claw.setPosition(wobble_claw_position);
        }

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twsist (rotating the whole chassis).
        double drive  = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double twist  = -gamepad1.left_stick_x;

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

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);

        // apply power to wobble arm
        wobble_arm.setPower(wobblePower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Thingy: ", "Wobble Arm Encoder " + wobble_arm.getCurrentPosition());
    }

}
