package org.firstinspires.ftc.teamcode.Auto;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareRobotMap;
import org.firstinspires.ftc.teamcode.SoftwareRobotMap;

@Autonomous(name="OneSampleAuto", group="Auto")
public class OneSampleAuto extends LinearOpMode {

    HardwareRobotMap robot = new HardwareRobotMap();

    private GoldAlignDetector detector;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = (50/62) ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        robot.ratchet.setPosition(0);

        telemetry.addLine("ready spaghetty");

        telemetry.update();

        waitForStart();

        //gyroTurn(90);

        robot.lift.setPower(-.56);
        sleep(12000);
        robot.lift.setPower(0);
        drive(-.7, 800);
        //gyroTurn(-90);
        turn(.5, 850);
        drive(.75, 850);

        while (!detector.isFound()) {
            //robot.leftBack.setPower(-0.5);
            robot.rightBack.setPower(-0.5);
            robot.leftFront.setPower(-0.5);
            //robot.rightFront.setPower(-0.5);
        }
        //robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        //robot.rightFront.setPower(0);

        sleep(250);

        //drive(.25, 400);

        //gyroTurn(0);
        turn(-.5, 850);//

        drive(-.25, 10107);

        //drive(.25, 2000);

        //gyroTurn(-90);
//        turn(-.5, 850);
//
//        drive(-.25, 400);
//
//
//        while (robot.sensorRange.getDistance(DistanceUnit.CM) >= 25) {
//            //robot.leftBack.setPower(.5);
//            robot.rightBack.setPower(.5);
//            robot.leftFront.setPower(.5);
//            //robot.rightFront.setPower(.5);
//        }
//
//        //robot.leftBack.setPower(0);
//        robot.rightBack.setPower(0);
//        robot.leftFront.setPower(0);
//        //robot.rightFront.setPower(0);
//
//        gyroTurn(135);
//
//        turn(0.5, 1350);
//
//        drive(1, 1500);
//
//        //robot.ratchet.setPosition(1);
//
//        sleep(500);
//
//        arm(.5,1500);
//

        //drive(-0.7, 4000);

        /*while (!detector.isFound()) {
            robot.leftBack.setPower(0.2);
            robot.rightBack.setPower(0.2);
            robot.leftFront.setPower(0.2);
            robot.rightFront.setPower(0.2);
        }
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);

        turn(-.5, 350);

        drive(.5, 300);

        robot.yee.setPosition(1);
    }
    */
    }
    public void Baddrive (double MotorPower, long time){
            //robot.leftBack.setPower(MotorPower);
            robot.rightBack.setPower(MotorPower);
            robot.leftFront.setPower(MotorPower);
            //robot.rightFront.setPower(MotorPower);
            sleep(time);
            //robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
            robot.leftFront.setPower(0);
            //robot.rightFront.setPower(0);
            sleep(500);
        }


    public void drive(double motorpower, double time){
        double speed;
        //was 0.0072
        double kp = 0.0075;

        float degrees = getHeading();

        ElapsedTime runTime = new ElapsedTime();

        float error = (degrees - getHeading());
        //run loop to turn
        while (time > runTime.milliseconds()){
            error = (degrees - getHeading());
            speed = kp * error;
            //robot.leftBack.setPower(motorpower - speed);
            robot.rightBack.setPower(motorpower + speed);
            robot.leftFront.setPower(motorpower - speed);
            //robot.rightFront.setPower(motorpower + speed);
        }
        //robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        //robot.rightFront.setPower(0);
        sleep(500);
    }
    public void turn(double MotorPower, long time){
        //robot.leftBack.setPower(-MotorPower);
        robot.rightBack.setPower(-MotorPower);
        robot.leftFront.setPower(MotorPower);
        //robot.rightFront.setPower(MotorPower);
        sleep(time);
        //robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        //robot.rightFront.setPower(0);
        sleep(500);

    }
    /*public void encoderDrive(double distance){
        //int lefttarget = (int) (robot.leftBack.getCurrentPosition() + distance*COUNTS_PER_INCH);
        int righttarget = (int) (robot.rightBack.getCurrentPosition() + distance*COUNTS_PER_INCH);

        double kp = 0.001;

        //float leftError = robot.leftBack.getCurrentPosition() - lefttarget;
        float rightError = robot.rightBack.getCurrentPosition() - righttarget;

        while(Math.abs(leftError) > 5 & Math.abs(rightError) > 5){
            leftError = robot.leftBack.getCurrentPosition() - lefttarget;
            rightError = robot.rightBack.getCurrentPosition() - righttarget;

            robot.leftFront.setPower(kp*lefttarget);
            robot.leftBack.setPower(kp*lefttarget);
            robot.rightFront.setPower(kp*righttarget);
            robot.rightBack.setPower(kp*righttarget);

            telemetry.addData("leftTarget", lefttarget);
            telemetry.addData("rightTarget", righttarget);
            telemetry.addData("leftError", leftError);
            telemetry.addData("rightError", rightError);
            telemetry.update();
        }

        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);
    }*/
    public float getHeading(){
        Orientation angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void arm(double power, long time){
        robot.intakeSweeper.setPower(power);
        sleep(time);
        robot.intakeSweeper.setPower(0);
    }
    public void gyroTurn (float degrees){

        //read orientation values from navx
        double speed;

        double kp = 0.008;

        ElapsedTime time5 = new ElapsedTime();

        time5.reset();

        float error = (degrees - getHeading());
        speed = kp * error;
        //run loop to turn
        while (Math.abs(error) > 5 && time5.milliseconds() < 3000){
            error = (degrees - getHeading());
            speed = kp * error;
            //robot.leftBack.setPower(speed);
            robot.rightBack.setPower(-speed);
            robot.leftFront.setPower(speed);
            //robot.rightFront.setPower(-speed);
        }
        //robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        //robot.rightFront.setPower(0);
        sleep(500);
    }
}
