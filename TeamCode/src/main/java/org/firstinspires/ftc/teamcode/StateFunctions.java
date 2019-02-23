package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class StateFunctions extends BasicFunctions {


    public void Deploy(){
        MoveLift(-1, 4400);
    }
    public void Sample(){
        drive(-.7, 800);
        gyroTurn(-90);
        //turn(.5, 850);
        drive(-.75, 850);

        while (!detector.isFound()) {
            //leftBack.setPower(-0.5);
            rightBack.setPower(0.6);
            leftFront.setPower(0.6);
            //rightFront.setPower(-0.5);
        }
        //leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        //rightFront.setPower(0);

        sleep(500);

        //drive(.25, 400);

        gyroTurn(-180);
        //turn(-.5, 850);//

        drive(.5, 1*1010.7);

        drive(-.45, 1*1010.7);
    }
    public void DriveToDepot(){
        while (sensorRange.getDistance(DistanceUnit.CM) >= 7.0101) {
            rightBack.setPower(.5);
            leftFront.setPower(.5);
        }

        //gyroTurn(315);
        gyroTurn(125);

        drive(-.8, 1.7*1010.7);
    }
    public void DropOffMarker(){
        intakeSweeper.setPower(1);
        sleep(1000);
        intakeSweeper.setPower(0);
    }
    public void ParkInCrater(){
        rightBack.setPower(1);
        leftFront.setPower(.9);
        sleep(2700);
        rightBack.setPower(0);
        leftFront.setPower(0);
    }
}
