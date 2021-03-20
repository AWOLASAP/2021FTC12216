// Copyright (c) 2017 FIRST. All rights reserved.

package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Backwards B Zone W/ Sounds", group="Autonomous")
public class BackwardsBZoneWSounds extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_right = null;
    private DcMotor front_left = null;
    private DcMotor back_right = null;
    private DcMotor back_left = null;

    //// Sound Stuff
    // List of available sound resources
    String  sounds[] =  {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
            "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
            "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie" };

    boolean soundPlaying = false;

    Context myApp = hardwareMap.appContext;
    //// Sound Stuff


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

    // Play Sound
    public void playSound(String sound, SoundPlayer.PlaySoundParams params) {
        int soundID = myApp.getResources().getIdentifier(sound, "raw", myApp.getPackageName());
        SoundPlayer.getInstance().startPlaying(myApp, soundID);
    }
    //// End of User Functions

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables.
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

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

        //// Sound Stuff
        // Create a sound parameter that holds the desired player parameters.
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;
        //// Sound Stuff

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

        // Let everyone know we mean business
        playSound("ss_power_up", params);

        // Move backwards into the B zone
        frontBack(-1, 2500);

        // Stop before moving over the line
        stopRobot();

        // Move forward over the line
        frontBack(1, 500);


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        // stop with button on controller
        if (gamepad1.b) {
            stop();
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

}
