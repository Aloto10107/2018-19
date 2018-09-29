package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class BasicFunctions extends OpMode {

    SoftwareRobotMap robot = new SoftwareRobotMap();

    public void dirveDistance(double distance){}

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
        while (Math.abs(resetC) > 0) {
            resetC = ((Math.abs(robot.leftFront.getCurrentPosition()) + Math.abs(robot.leftBack.getCurrentPosition()) + Math.abs(robot.rightFront.getCurrentPosition()) + Math.abs(robot.rightBack.getCurrentPosition())));
            //idle();
        }
        // switch the motors back to RUN_USING_ENCODER mode
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}