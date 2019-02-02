
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="GROUND_WHY", group="Pushbot")
public class GROUND_WHY extends LinearOpMode {

    private Spaceboy robot = new Spaceboy();   // Use a Pushbot's hardware

    private ElapsedTime runtime = new ElapsedTime();
    private Orientation lastAngles = new Orientation();
    private double globalAngle, power = .30, correction;

    static final double COUNTS_PER_MOTOR_REV = 1120 / 2;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.5;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double HOOK_PER_MOTOR_REV = 460;
    static final double AXLE_DIAMETER_INCHES = 0.35;
    static final double HOOK_COUNTS_PER_INCH = (HOOK_PER_MOTOR_REV * 1 /
            (AXLE_DIAMETER_INCHES * Math.PI));
    static final double DRIVE_SPEED = 0.20;
    static final double TURN_SPEED = 0.1;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        robot.imu.initialize(parameters);

        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.update();

        waitForStart();

        //this is our new lower hook
        //encoderHook(30, 27, 10);  //Hook going up is POSITIVE distance

        test();

        //resetHook();
        telemetry.update();
    }

    private void test() {
        //lowerRobot();
        if (robot.autoSelect.isPressed()) { // NW/SE //Down into the crater
            driveRobot(14.5); //16.5
            spinRobot(75);  //90
            driveRobot(45);
            spinRobot(65);
            driveRobot(40.5);
            dropToken();
            driveRobot(-93);
        } //else { // NE/SW //Up to the depot
        //driveRobot(16);
        //spinRobot(80);
        //driveRobot(47);
        //spinRobot(-122.5);
        //driveRobot(49);
        //dropToken();
        //driveRobot(-73);
        else { // NE/SW //Up to the depot
            driveRobot(18);
            spinRobot(-95);
            driveRobot(-50);
            spinRobot(51);
            driveRobot(49);
            dropToken();
            //spinRobot(180);
            driveRobot(-73);
        }


    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = 0.01;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, - is left + is right
     */
    private void rotate(double degrees, double power) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();
        if(opModeIsActive()) {
            // getAngle() returns + when rotating counter clockwise (left) and - when rotating
            // clockwise (right).

            if (degrees < 0) {   // turn right.
                leftPower = power;
                rightPower = -power;
            } else if (degrees > 0) {   // turn left.
                leftPower = -power;
                rightPower = power;
            } else return;

            // set power to rotate.
            robot.motorLeft.setPower(leftPower);
            robot.motorRight.setPower(rightPower);

            // rotate until turn is completed.
            if (degrees < 0) {
                // On right turn we have to get off zero first.
                while (opModeIsActive() && getAngle() == 0) {
                    telemetry.addData("right turn off 0", getAngle());
                    telemetry.update();
                }

                while (opModeIsActive() && getAngle() > degrees) {
                    telemetry.addData("right turn", getAngle());
                    telemetry.update();
                }
            } else {    // left turn.
                while (opModeIsActive() && getAngle() < degrees) {
                    telemetry.addData("left turn", getAngle());
                    telemetry.update();
                }
            }

            // turn the motors off.
            robot.motorRight.setPower(0);
            robot.motorLeft.setPower(0);

            double delta = Math.abs(getAngle() - degrees);
            if (delta > 0.5) {
                rotate((degrees - getAngle()), 0.02);
            }

            // reset angle tracking on new heading.
            resetAngle();
        }
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);
    }

    public void encoderHook(double speed,
                            double inches,
                            double timeoutS) {
        int newTarget;

        if (opModeIsActive()) {
            newTarget = robot.liftoffHook.getCurrentPosition() + (int) (inches * HOOK_COUNTS_PER_INCH);
            robot.liftoffHook.setTargetPosition(newTarget);

            robot.liftoffHook.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.liftoffHook.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.liftoffHook.isBusy())) {

            }

            robot.liftoffHook.setPower(0);

        }
    }

    private void spinRobot(double degrees) {
        rotate((int) degrees, TURN_SPEED);
        //double circumference = 15.0 * Math.PI;
        //double distance = degrees/360.0 * circumference;

        //encoderDrive (TURN_SPEED, distance, -distance, 5);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {

            newLeftTarget = (robot.motorLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH));
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
                //if (robot.distanceSensor.getDistance(DistanceUnit.CM) > 5){

                if (leftInches > 0 && rightInches > 0) {
                    correction = checkDirection();
                    robot.motorLeft.setPower(Math.abs(speed) - correction);
                    robot.motorRight.setPower(Math.abs(speed));
                } else if (leftInches < 0 && rightInches < 0) {
                    correction = checkDirection();
                    robot.motorLeft.setPower(Math.abs(speed) + correction);
                    robot.motorRight.setPower(Math.abs(speed));
                }

                //}else {
                //robot.motorLeft.setPower(0);
                //robot.motorRight.setPower(0);
            }
            telemetry.addData("current angle", getAngle());
            telemetry.addData("correction", correction);
            //telemetry.addData("range", String.format("%.01f mm", robot.distanceSensor.getDistance(DistanceUnit.MM)));
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.motorLeft.getCurrentPosition(),
                    robot.motorRight.getCurrentPosition());
            telemetry.update();
        }
        robot.motorLeft.setPower(0);
        robot.motorRight.setPower(0);

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    private void driveRobot(double distance) {
        encoderDrive(DRIVE_SPEED, distance, distance, 10);
    }
    private void dropToken() {

        robot.tokenDrop.setPosition(1);
        try{
            Thread.sleep(2000);
        }catch(InterruptedException e){
            System.out.println("got interrupted!");
        }
        robot.tokenDrop.setPosition(0.2);
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
    public void lowerRobot() {
        encoderHook(.75, 40, 10);  //Hook going up is POSITIVE distance
    }
}
