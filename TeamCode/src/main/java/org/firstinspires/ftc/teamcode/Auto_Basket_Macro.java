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

@Autonomous(name="Auto_Basket_Macro")
public class Auto_Basket_Macro extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor turretLeft = null;
    private DcMotor turretRight = null;
    private DcMotor rightSlide = null;
    private DcMotor leftSlide = null;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo LeftClaw = null;
    private Servo RightClaw = null;

    private double power_adjust = 1.0;

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
        LeftClaw.setDirection(Servo.Direction.FORWARD);
        RightClaw.setDirection(Servo.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        turretLeft.setDirection(DcMotor.Direction.FORWARD);
        turretRight.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);

        turretLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //this should help with battery voltage issues
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        // close claw so the preloaded specimen does not fall out
        CloseClaw();

        // Wait for the game to start (driver presses START)
        waitForStart();

        Strafe(0.5, 400);
        PositionTurret(3000, .8);
        sleep(1000);
        PositionSlide(1800, 0.8);
        sleep(1000);
        Drive(.3, 500);
        OpenClaw();
        sleep(300);

       // lifts the arm a bit so it doesn't get stuck on the basket
        PositionTurret(3100,0.8);
        sleep(100);
        PositionSlide(400,0.8);
        Drive(-0.3,500);
        Turn(0.8,630);
        // align with the wall
        Strafe(-0.5,800);
        //goes to middle sample
        //Drive(0.5, 200);
        PositionTurret(290,1.0);
        sleep(1000);
        Strafe(0.3,340);
        PositionSlide(1300,0.8);
        sleep(500);
        CloseClaw();
        sleep(500);

        PositionTurret(3000, 0.8);
        sleep(1000);
        PositionSlide(1800, 0.8);
        sleep(1000);
        Turn(0.8, 1000);
        Drive(.3, 500);
        OpenClaw();
        sleep(300);
        PositionTurret(3100, 0.8);
        sleep(100);
        Drive(-.3, 300);
        Turn(-0.8,700);
        sleep(200);
        PositionTurret(330,1.0);
        sleep(1000);
        PositionSlide(1350,0.8);
        sleep(1000);
        //Drive(0.3,500);
        CloseClaw();

        sleep(1000);
        PositionTurret(3000,0.8);

    }

    // -------------- Helper functions -------------
    public void OpenClaw() {
        double openClawPosition = 0.015;
        LeftClaw.setPosition(openClawPosition);
        RightClaw.setPosition(openClawPosition);
    }

    public void CloseClaw() {
        double closedClawPosition = 0.04;
        LeftClaw.setPosition(closedClawPosition);
        RightClaw.setPosition(closedClawPosition);
    }
    public void Strafe(double power, int duration) {
        // positive power strafes right
        leftBackDrive.setPower(-power);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(power);
        sleep((int)(duration * power_adjust));
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void Drive(double power, int duration) {
        //positive drives forward
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        sleep((int)(duration * power_adjust));
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void Turn(double power, int duration) {
        // need to fix these for turning
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);
        sleep((int)(duration * power_adjust));
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void MoveTurret(double power, int duration) {
        turretLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretLeft.setPower(power);
        turretRight.setPower(power);
        sleep((int)(duration * power_adjust));
        turretLeft.setPower(0);
        turretRight.setPower(0);
    }

    public void MoveSlide(double power, int duration) {
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setPower(power);
        leftSlide.setPower(power);
        sleep((int)(duration * power_adjust));
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
