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
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Start Hook", group="Pushbot")
public class Start_Hook extends LinearOpMode {

    /* Declare OpMode members. */
    private Spaceboy robot   = new Spaceboy();   // Use a Pushbot's hardware

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     HOOK_PER_MOTOR_REV      = 420;
    static final double     AXLE_DIAMETER_INCHES    = 0.35;
    static final double     HOOK_COUNTS_PER_INCH    =(HOOK_PER_MOTOR_REV * 1/ (AXLE_DIAMETER_INCHES * Math.PI));
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.motorLeft.getCurrentPosition(),
                          robot.motorRight.getCurrentPosition(),
                          robot.liftoffHook.getCurrentPosition());

        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        // lower the robot

        resetHook();
        encoderHook(5,14,10);
        //if(true){  // THIS IS WHERE THE LOGIC GOES FOR WHERE WE ARE!
            //encoderHook(.75,20,5);
            //stop();
        //}else{
            //resetHook();
        //}

        //parkRobot();
        //encoderDrive(DRIVE_SPEED, 48, 48, 10);
        telemetry.update();
    }

    public void encoderHook(double speed,
                             double inches,
                             double timeoutS) {
        int newTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newTarget = robot.liftoffHook.getCurrentPosition() + (int)(inches * HOOK_COUNTS_PER_INCH);
            robot.liftoffHook.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            robot.liftoffHook.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.liftoffHook.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.liftoffHook.isBusy())){

                // Display it for the driver.
                telemetry.addData("Path",  "To %7d : current: %7d", newTarget, robot.liftoffHook.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            robot.liftoffHook.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.liftoffHook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
    private void spinRobot (double degrees){
        //calculate circumference

        double circumference = 15.0 * Math.PI;

        //calculate distance
        double distance = degrees/360.0 * circumference;

        encoderDrive (TURN_SPEED, distance, -distance, 5);
    }
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = (robot.motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            newRightTarget = (robot.motorRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH));
            robot.motorLeft.setTargetPosition(newLeftTarget);
            robot.motorRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorLeft.setPower(Math.abs(speed));
            robot.motorRight.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.motorLeft.isBusy() && robot.motorRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.motorLeft.getCurrentPosition(),
                                            robot.motorRight.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    private void parkRobot() {
        /*
        lower*/
        /*lowerRobot();*/
        lowerRobot();
        /*spin 180 degrees*/
        /*spinRobot(180);*/
        encoderDrive(DRIVE_SPEED,0, 10,5);
        /*move backwards 20 inches*/
        encoderDrive(DRIVE_SPEED,-20,-20,5);
        /*turn left 100 degrees*/
        spinRobot(-100);
        /* move forward 18 inches*/
        encoderDrive(DRIVE_SPEED,18,18,5);

        /*detect picture*/
        //VuforiaTrackable location = roverNav.getCurrentTrackable();

        if(true){  // THIS IS WHERE THE LOGIC GOES FOR WHERE WE ARE!
            /* move forward 18 inches*/
            encoderDrive(DRIVE_SPEED,18,18,5);
            /*Crater/BackSpace means turn left 120 degrees*/
            spinRobot(-120);
        }else{
            /* move forward 18 inches*/
            encoderDrive(DRIVE_SPEED,18,18,5);
            /*Foot/Rover means turn right 30 degrees*/
            spinRobot(30);
        }
        /*determine which direction to turn depending on picture seen*/

        /*move forward 24 inches */
        encoderDrive(DRIVE_SPEED,36,36,5);
    }
    private void resetHook() {
        robot.liftoffHook.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftoffHook.setPower(-25);
        while (!robot.hookTouch.isPressed()) {

        }
        robot.liftoffHook.setPower(0);
        robot.liftoffHook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void lowerRobot() {
        /*encoderHook(0.5, -1,5);*/
        encoderHook(.75, 3,10);  //Hook going up is POSITIVE distance
    }
}
