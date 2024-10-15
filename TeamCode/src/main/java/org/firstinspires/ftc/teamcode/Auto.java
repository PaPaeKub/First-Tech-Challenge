package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilize.AtTargetRange;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@Autonomous(name="Auto", group = "In")
public class Auto extends Robot {
    private char signalPos;

    private void Init() {
        // Initialize Robot
        Initialize(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new double[]{0, 0, 0},
                new double[]{0.3, 0, 0.1, 0});
        imu.resetYaw();

    }

    private void WaitForStart() {
        while (!isStarted() && !isStopRequested()) {
            Posx = 0;
            Posy = 0;
        }

    }

    public void runOpMode() {
        Init();
        WaitForStart();
        if (opModeIsActive()) {
            move(0.04, 2.0, 2.0, 0.0,0.063, 0.04, 0.000000005, 0);
            move(0.04, 0.0, 0.0, 0.0,0.063, 0.04, 0.000000005, 0);
            move(0.04, 4.0, 3.0, 0.0,0.063, 0.04, 0.000000005, 0);
            move(0.04, 0.0, 3.0, 0.0,0.043, 0.04, 0.000000005, 0);
            move(0.04, 0.0, 0.0, 0.0,0.015, 0.01, 0.000000005, 0);
            move(0.04, 3.0, 0.0, 0.0,0.043, 0.01, 0.000000002, 0);
            move(0.04, 0.0, 0.0, 0.0,0.043, 0.01, 0.000000002, 0);
            sleep(1000000000);

        }

    }
}
