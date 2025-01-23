/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Auto_Specimen")
public class Auto_Specimen extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor turretLeft = null;
    private DcMotor turretRight = null;
    private DcMotor rightSlide = null;
    private DcMotor leftSlide = null;
    private TouchSensor touchSensor_left = null;
    private TouchSensor touchSensor_right = null;
    double turret_speed;

    private ElapsedTime     runtime = new ElapsedTime();


    private Servo LeftClaw = null;
    private Servo RightClaw = null;
    private double openClawPosition = 0.01;
    private double closedClawPosition = 0.04;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "Frontleft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "Backleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Frontright");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Backright");
        turretLeft = hardwareMap.get(DcMotor.class, "Turret_Left");
        turretRight = hardwareMap.get(DcMotor.class, "Turret_Right");
        rightSlide = hardwareMap.get(DcMotor.class, "Rightslide");
        leftSlide = hardwareMap.get(DcMotor.class, "Leftslide");
        LeftClaw = hardwareMap.get(Servo.class, "LeftRoller");
        RightClaw = hardwareMap.get(Servo.class, "RightRoller");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        turretRight.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        RightClaw.setDirection(Servo.Direction.REVERSE);


        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();

        // Consider replacing with a method call
        //OpenClaw();
        LeftClaw.setPosition(closedClawPosition);
        RightClaw.setPosition(closedClawPosition);

        //Consider replacing the block below with
        //MoveTurret(1.0, 0.75);

        //Move Turret Up
        turretLeft.setPower(1);
        turretRight.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.75)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        turretLeft.setPower(0.0);
        turretRight.setPower(0.0);

        // Consider replacing with a method call
        //DriveForward(1.0, 0.4);

        //Move Forwards
        leftFrontDrive.setPower(1);
        leftBackDrive.setPower(1);
        rightFrontDrive.setPower(1);
        rightBackDrive.setPower(1);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // this seems redundant
        leftBackDrive.setPower(1);
        leftFrontDrive.setPower(-1);
        rightFrontDrive.setPower(1);
        rightBackDrive.setPower(-1);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        leftFrontDrive.setPower(1);
        leftBackDrive.setPower(1);
        rightFrontDrive.setPower(1);
        rightBackDrive.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        leftBackDrive.setPower(0.0);
        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);

        sleep(500);

//Extend Slides
        leftFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);

        leftSlide.setPower(0.7);
        rightSlide.setPower(0.7);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        leftSlide.setPower(0.1);
        rightSlide.setPower(0.1);
        sleep(500);

        LeftClaw.setPosition(openClawPosition);
        RightClaw.setPosition(openClawPosition);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.6)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        leftSlide.setPower(0.0);
        rightSlide.setPower(0.0);
        sleep(500);

//Move Turret back down
        turretLeft.setPower(-0.7);
        turretRight.setPower(-0.7);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        turretLeft.setPower(0.0);
        turretRight.setPower(0.0);
        //sleep(500);


        leftSlide.setPower(-0.5);
        rightSlide.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        leftSlide.setPower(0.0);
        rightSlide.setPower(0.0);
        //sleep(500);
        /*

//Move Forward
        leftFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);
        sleep(500);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
//Strafe Right
        leftBackDrive.setPower(-0.59);
        leftFrontDrive.setPower(0.59);
        rightFrontDrive.setPower(-0.59);
        rightBackDrive.setPower(0.59);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
//Move Backwards
        leftFrontDrive.setPower(-0.5);
        leftBackDrive.setPower(-0.5);
        rightFrontDrive.setPower(-0.5);
        rightBackDrive.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.3)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
//Move Forwards
        leftFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.3)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
//Strafe Right

        leftBackDrive.setPower(-0.59);
        leftFrontDrive.setPower(0.59);
        rightFrontDrive.setPower(-0.59);
        rightBackDrive.setPower(0.59);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
//Move Backwards
        leftFrontDrive.setPower(-0.5);
        leftBackDrive.setPower(-0.5);
        rightFrontDrive.setPower(-0.5);
        rightBackDrive.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.3)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
//Move Forwards
        leftFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.3)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
//Strafe Right
        leftBackDrive.setPower(-0.59);
        leftFrontDrive.setPower(0.59);
        rightFrontDrive.setPower(-0.59);
        rightBackDrive.setPower(0.59);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
//Move Backwards
        leftFrontDrive.setPower(-0.5);
        leftBackDrive.setPower(-0.5);
        rightFrontDrive.setPower(-0.5);
        rightBackDrive.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.3)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        */

        // Step 4:  Stop
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    public void OpenClaw() {
        LeftClaw.setPosition(openClawPosition);
        RightClaw.setPosition(openClawPosition);
    }

    public void CloseClaw() {
        LeftClaw.setPosition(closedClawPosition);
        RightClaw.setPosition(closedClawPosition);
    }

    public void StrafeRight(double power, double duration) {
        // positive power strafes right
        leftBackDrive.setPower(-power);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(power);
        sleep((int)(duration * 1000));
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void DriveForward(double power, double duration) {
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        sleep((int) (duration * 1000));
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void TurnRight(double power, double duration) {
        // need to fix these for turning
        leftBackDrive.setPower(-power);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(power);
        sleep((int) (duration * 1000));
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void MoveTurret(double power, double duration) {
        turretLeft.setPower(power);
        turretRight.setPower(power);
        sleep((int) (duration * 1000));
        turretLeft.setPower(0);
        turretRight.setPower(0);
    }

    public void MoveSlide(double power, double duration) {
        rightSlide.setPower(power);
        leftSlide.setPower(power);
        sleep((int) (duration * 1000));
        rightSlide.setPower(0);
        leftSlide.setPower(0);
    }

    public void PositionTurret(int position, double power) {
        //Function to move arm to a specific angle
        turretLeft.setTargetPosition(position);
        turretRight.setTargetPosition(position);
        turretLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretLeft.setPower(power);
        turretRight.setPower(power);
    }

    public void PositionSlide(int position, double power) {
        // Function to move slide to a position and hold
        rightSlide.setTargetPosition(position);
        leftSlide.setTargetPosition(position);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setPower(power);
        leftSlide.setPower(power);
    }


}
