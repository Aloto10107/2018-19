package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="driveTest", group="Auto")
public class DriveEncoder extends LinearOpMode {
    SoftwareRobotMap robot = new SoftwareRobotMap();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.addLine("ready spaghetty");

        telemetry.update();

        waitForStart();

/*        robot.leftFront.setPower(Math.abs(.5));
        robot.rightFront.setPower(Math.abs(.5));
        robot.leftBack.setPower(Math.abs(.5));
        robot.rightBack.setPower(Math.abs(.5));
        sleep(3000);
        robot.leftFront.setPower(Math.abs(.5));
        robot.rightFront.setPower(Math.abs(.5));
        robot.leftBack.setPower(Math.abs(.5));
        robot.rightBack.setPower(Math.abs(.5));*/

        encoderDrive(.5, 24, 24, 10);


        //encoderDrive(.50,.50,48, 20, .75);

/*        robot.leftFront.setPower(.5);
    robot.rightFront.setPower(.5);
    robot.leftBack.setPower(.5);
    robot.rightBack.setPower(.5);
leftFront
    sleep(500);/* BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
    //parameters.loggingEnabled      = true;
    //parameters.loggingTag          = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    imu = hwMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);*/

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newRightBackTarget;
        int newLeftBackTarget;
        ElapsedTime runtime = new ElapsedTime();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = robot.leftBack.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBack.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFront.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
            robot.leftFront.setTargetPosition(newLeftFrontTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);
            robot.rightBack.setTargetPosition(newRightBackTarget);
            robot.leftBack.setTargetPosition(newLeftBackTarget);


            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFront.setPower(speed);
            robot.rightFront.setPower(speed);
            robot.leftBack.setPower(speed);
            robot.rightBack.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.rightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("rightFront", robot.rightFront.getCurrentPosition());
                telemetry.addData("leftBack", robot.leftBack.getCurrentPosition());
                telemetry.addData("leftFront", robot.leftFront.getCurrentPosition());
                telemetry.addData("rightBack", robot.rightBack.getCurrentPosition());
                telemetry.addData("rightFrontTarget", newRightFrontTarget);
                telemetry.addData("rightBackTarget", newRightBackTarget);
                telemetry.addData("leftFrontTarget", newLeftFrontTarget);
                telemetry.addData("leftBackTarget", newLeftBackTarget);

                telemetry.update();
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}



//public void runOpMode() {
//
//        /*
//         * Initialize the drive system variables.
//         * The init() method of the hardware class does all the work here
//         */
//        robot.init(hardwareMap);
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Status", "Resetting Encoders");    //
//        telemetry.update();
//
//        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//  }
//}
