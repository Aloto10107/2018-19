package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SoftwareRobotMap {
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor leftFront = null;
    public DcMotor rightBack = null;

    HardwareMap hwMap =  null;


    /* Constructor */
    public SoftwareRobotMap(){

    }

    public void init(HardwareMap ahwmap){

        hwMap =  ahwmap;
        leftFront  = hwMap.get(DcMotor.class, "left_Front");
        rightBack = hwMap.get(DcMotor.class, "right_Back");
        leftBack    = hwMap.get(DcMotor.class, "left_Back");
        rightFront = hwMap.get(DcMotor.class, "right_Front");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
}