package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.StateFunctions;

@Autonomous(name="TestAbstractionAuto", group="Auto")
public class TestAbstractionAuto extends StateFunctions {

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);

        telemetry.addLine("ready spaghetty");

        telemetry.update();

        waitForStart();

        Deploy();

        Sample();

        gyroTurn(-96);

        DriveToDepot();

        DropOffMarker();

        ParkInCrater();
    }
}
