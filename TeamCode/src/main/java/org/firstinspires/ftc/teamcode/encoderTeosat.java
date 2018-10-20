package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="encoderTest", group="Auto")
public class encoderTeosat extends LinearOpMode {
    SoftwareRobotMap robot = new SoftwareRobotMap();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        encoderDrive(50, 50, 24, 10, 10);

        gyroTurn(90);


        //encoderDrive(.50,.50,48, 20, .75);

/*        robot.leftFront.setPower(.5);
        robot.rightFront.setPower(.5);
        robot.leftBack.setPower(.5);
        robot.rightBack.setPower(.5);

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

    public void encoderDrive(double Lspeed, double Rspeed, double Inches, double timeoutS, double rampup) throws InterruptedException {

        double COUNTS_PER_MOTOR_REV = 560;    //Set for NevRest 20 drive. For 40's change to 1120. For 60's 1680
        double DRIVE_GEAR_REDUCTION = 1.0;     // This is the ratio between the motor axle and the wheel
        double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        ElapsedTime runtime = new ElapsedTime();

        //initialise some variables for the subroutine
        int newLeftTarget;
        int newRightTarget;
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
        newLeftTarget = (robot.leftFront.getCurrentPosition() + robot.leftBack.getCurrentPosition()) / 2 + (int) (Inches * COUNTS_PER_INCH);
        newRightTarget = (robot.rightFront.getCurrentPosition() + robot.rightBack.getCurrentPosition()) / 2 + (int) (Inches * COUNTS_PER_INCH);
        // reset the timeout time and start motion.
        runtime.reset();
        // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
        while ((runtime.seconds() < timeoutS) &&
                (Math.abs(robot.leftFront.getCurrentPosition() + robot.leftBack.getCurrentPosition()) / 2 < newLeftTarget &&
                        Math.abs(robot.rightFront.getCurrentPosition() + robot.rightBack.getCurrentPosition()) / 2 < newRightTarget)) {

            double rem = (Math.abs(robot.leftFront.getCurrentPosition()) + Math.abs(robot.leftBack.getCurrentPosition()) + Math.abs(robot.rightFront.getCurrentPosition()) + Math.abs(robot.rightBack.getCurrentPosition())) / 4;
            double NLspeed;
            double NRspeed;
            //To Avoid spinning the wheels, this will "Slowly" ramp the motors up over
            //the amount of time you set for this SubRun
            double R = runtime.seconds();
            if (R < rampup) {
                double ramp = R / rampup;
                NLspeed = Lspeed * ramp;
                NRspeed = Rspeed * ramp;
            }
//Keep running until you are about two rotations out
            else if (rem > (1000)) {
                NLspeed = Lspeed;
                NRspeed = Rspeed;
            }
            //start slowing down as you get close to the target
            else if (rem > (200) && (Lspeed * .2) > .1 && (Rspeed * .2) > .1) {
                NLspeed = Lspeed * (rem / 1000);
                NRspeed = Rspeed * (rem / 1000);
            }
            //minimum speed
            else {
                NLspeed = Lspeed * .2;
                NRspeed = Rspeed * .2;

            }
            //Pass the seed values to the motors
            robot.leftFront.setPower(NLspeed);
            robot.leftBack.setPower(NLspeed);
            robot.rightFront.setPower(NRspeed);
            robot.rightBack.setPower(NRspeed);
        }
        // Stop all motion;
        //Note: This is outside our while statement, this will only activate once the time, or distance has been met
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        // show the driver how close they got to the last target
        telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
        telemetry.addData("Path2", "Running at %7d :%7d", robot.leftFront.getCurrentPosition(), robot.rightFront.getCurrentPosition());
        telemetry.update();
        //setting resetC as a way to check the current encoder values easily
        double resetC = ((Math.abs(robot.leftFront.getCurrentPosition()) + Math.abs(robot.leftBack.getCurrentPosition()) + Math.abs(robot.rightFront.getCurrentPosition()) + Math.abs(robot.rightBack.getCurrentPosition())));
        //Get the motor encoder resets in motion
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //keep waiting while the reset is running
        /*while (Math.abs(resetC) > 0) {
            resetC = ((Math.abs(robot.leftFront.getCurrentPosition()) + Math.abs(robot.leftBack.getCurrentPosition()) + Math.abs(robot.rightFront.getCurrentPosition()) + Math.abs(robot.rightBack.getCurrentPosition())));
            //idle();
        }*/
        // switch the motors back to RUN_USING_ENCODER mode
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(250);
    }

    public float getHeading(){
        Orientation angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void gyroTurn (float degrees){

        //read orientation values from navx
        double speed;

        float kp = (float)0.008;

        float error = degrees - getHeading();
        //run loop to turn
        while (Math.abs(error) > 5){
            error = degrees - getHeading();
            speed = kp * error;
            robot.leftBack.setPower(-speed);
            robot.rightBack.setPower(speed);
            robot.leftFront.setPower(-speed);
            robot.rightFront.setPower(speed);
        }
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
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
//        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//  }




