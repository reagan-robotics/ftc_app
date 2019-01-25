
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="SERVO_CHANGES", group="Pushbot")
public class SERVO_SHIT extends LinearOpMode {

    private Spaceboy robot   = new Spaceboy();   // Use a Pushbot's hardware
    private RoverNav roverNav = null;
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     HOOK_PER_MOTOR_REV      = 420;
    static final double     AXLE_DIAMETER_INCHES    = 0.35;
    static final double     HOOK_COUNTS_PER_INCH    =(HOOK_PER_MOTOR_REV * 1/
            (AXLE_DIAMETER_INCHES * Math.PI));
    static final double     DRIVE_SPEED             = 0.45;
    static final double     TURN_SPEED              = 0.30;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.update();

        waitForStart();
        dropToken();

        //resetHook();
        telemetry.update();
    }

    public void encoderHook(double speed,
                            double inches,
                            double timeoutS) {
        int newTarget;

        if (opModeIsActive()) {
            newTarget = robot.liftoffHook.getCurrentPosition() + (int)(inches * HOOK_COUNTS_PER_INCH);
            robot.liftoffHook.setTargetPosition(newTarget);

            robot.liftoffHook.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.liftoffHook.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.liftoffHook.isBusy())){

            }

            robot.liftoffHook.setPower(0);

        }
    }
    private void spinRobot (double degrees){

        double circumference = 15.0 * Math.PI;
        double distance = degrees/360.0 * circumference;

        encoderDrive (TURN_SPEED, distance, -distance, 5);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {

            newLeftTarget = (robot.motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            newRightTarget = (robot.motorRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH));
            robot.motorLeft.setTargetPosition(newLeftTarget);
            robot.motorRight.setTargetPosition(newRightTarget);

            robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.motorLeft.setPower(Math.abs(speed));
            robot.motorRight.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorLeft.isBusy() && robot.motorRight.isBusy())) {

                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.motorLeft.getCurrentPosition(),
                        robot.motorRight.getCurrentPosition());
                telemetry.update();
            }
            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);

            robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    private void driveRobot(double distance) {
        encoderDrive(DRIVE_SPEED, distance, distance, 5);
    }
    private void dropToken() {

        robot.tokenDrop.setPosition(0.5);
        try{
            Thread.sleep(2000);
        }catch(InterruptedException e){
            System.out.println("got interrupted!");
        }
        robot.tokenDrop.setPosition(0.75);
    }
    private void resetHook()
    {
        robot.liftoffHook.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftoffHook.setPower(-25);
        while (!robot.hookTouch.isPressed()) {

        }
        robot.liftoffHook.setPower(0);
        robot.liftoffHook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



            //dropToken();

  //      dropToken();
  //      spinRobot(45);
  //      spinRobot(-187);
  //      driveRobot(80);
        //resetHook();
    }

