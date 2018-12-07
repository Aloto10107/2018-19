package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="teleop", group="teleop")
public class Teleop extends OpMode {

    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor leftFront = null;
    public DcMotor rightBack = null;
    public BNO055IMU imu = null;
    public DcMotor lift = null;
    public Servo ratchet = null;

    double rightPower = 0;
    double leftPower = 0;

    public DistanceSensor sensorRange;

    @Override
    public void init() {
        leftFront  = hardwareMap.get(DcMotor.class, "left_Front");
        rightBack = hardwareMap.get(DcMotor.class, "right_Back");
        leftBack    = hardwareMap.get(DcMotor.class, "left_Back");
        rightFront = hardwareMap.get(DcMotor.class, "right_Front");
        lift = hardwareMap.get(DcMotor.class, "lift_Arm");
        ratchet = hardwareMap.get(Servo.class, "ratchet");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        /*lift = hardwareMap.get(DcMotor.class, "hookArm");
        lift.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //parameters.loggingEnabled      = true;
        //parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    @Override
    public void loop() {

        rightPower = ((-gamepad1.right_trigger + gamepad1.left_trigger)  + (gamepad1.left_stick_x));
        leftPower = ((-gamepad1.right_trigger + gamepad1.left_trigger)  - (gamepad1.left_stick_x));

        /*rightPower = -gamepad1.right_stick_y;
        `
        leftPower = -gamepad1.left_stick_y;*/

        if (Math.abs(leftPower) < .2){
            leftPower = 0;
        }
        if (Math.abs(rightPower) < .2){
            rightPower = 0;
        }

        rightPower = 0.7*rightPower;
        leftPower = 0.7*leftPower;

        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);
        leftBack.setPower(leftPower);
        leftFront.setPower(leftPower);

        if(gamepad2.right_bumper && !gamepad2.left_bumper){
            lift.setPower(1);
        }

        if(!gamepad2.right_bumper && !gamepad2.left_bumper){
            lift.setPower(0);
        }

        if(gamepad2.left_bumper && !gamepad2.right_bumper){
            lift.setPower(-1);
            }

        if(gamepad2.a){
            ratchet.setPosition(1);
        }
        if(!gamepad2.a){
            ratchet.setPosition(0);

        }

        telemetry.addData("heading", getHeading());
        telemetry.addData("rightFront", rightFront.getCurrentPosition());
        telemetry.addData("leftFront", leftFront.getCurrentPosition());
        telemetry.addData("rightBack", rightBack.getCurrentPosition());
        telemetry.addData("leftBack", leftBack.getCurrentPosition());
        telemetry.addData("distance", sensorRange.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
    public float getHeading(){
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}
