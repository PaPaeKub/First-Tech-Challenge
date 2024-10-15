package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilize.WrapRads;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;

public abstract class Robot extends LinearOpMode {
    public IMU imu;
    public VisionPortal visionPortal;
    public Servo LA, RA, LH, RH, ALL, ARL, IT, DP, ADP;
    public DcMotorEx FL, FR, BL, BR, RL, LL,  encoder1, encoder2, encoder3 ;
    public int FL_Target, FR_Target, BL_Target, BR_Target;
    public final double[] tileSize            = {60.0, 60.0};  // Width * Length
    /* TETRIX Motor Encoder per revolution */
    public final int      Counts_per_TETRIX   = 24;
    /** HD HEX Motor Encoder per revolution */
    public final int      Counts_per_HD_HEX   = 28;
    /** 20:1 HD HEX Motor Encoder per revolution */
    public final int      Gear_20_HD_HEX      = Counts_per_HD_HEX * 20;
    /** (3 * 4 * 5):1 UltraPlanetary HD HEX Motor Encoder per revolution */
    public final double   Gear_60_HD_HEX      = Counts_per_HD_HEX * 54.8;
    public final double   Wheel_Diameter_Inch = 7.5/2.54;
    public final double   Counts_per_Inch     = Gear_20_HD_HEX / (Wheel_Diameter_Inch * Math.PI);
    public double[]       currentXY           = {0, 0};
    public final double   L                   = 33.5; //distance between 1 and 2 in cm
    public final double   B                   = 9.5; //distance between center of 1 and 2 and 3 in cm
    public final double   r                   = 2.4 ; // Odomentry wheel radius in cm
    public final double   N                   = 2000.0 ; // ticks per one rotation
    public double         cm_per_tick     = 2.0 * Math.PI * r / N ;
    public double         theta, Posx, Posy, heading, n, CurPosLift, Lift_Power, dn1, dn2, dn3, dyaw ;
    private Controller    controller;
    //    // update encoder
    int                  Current1Position, Current2Position, Current3Position, Old1Position,
                         Old2Position, Old3Position = 0;
    double               CurrentYaw, OldYaw         = 0;


    public void Odomentry() {
        Current1Position = -encoder1.getCurrentPosition();
        Current2Position = -encoder2.getCurrentPosition();
        Current3Position = encoder3.getCurrentPosition();
        CurrentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double alpha = 0.8;
        dn1 = alpha * dn1 + (1 - alpha) * (Old1Position - Current1Position) * cm_per_tick;
        dn2 = alpha * dn2 + (1 - alpha) * (Old2Position - Current2Position) * cm_per_tick;
        dn3 = (Current3Position - Old3Position) * cm_per_tick;
        dyaw = CurrentYaw - OldYaw;



        double dy = (dn1 + dn2) / 2;
        double dx = dn3 - B * dyaw;

        double deltaY = dy * Math.cos(CurrentYaw) - dx * Math.sin(CurrentYaw);
        double deltaX = dy * Math.sin(CurrentYaw) + dx * Math.cos(CurrentYaw);

        Posy -= deltaY;
        Posx += deltaX;
        theta += (dn1 - dn2) / L;

        Old1Position = Current1Position;
        Old2Position = Current2Position;
        Old3Position = Current3Position;
        OldYaw = CurrentYaw;
    }

    public void move(double power, double tilex, double tiley , double setpoint, double Kp, double Ki, double Kd, double Kf){
        Controller  pidR    = new Controller(5.0, 0.01, 0.09, 0);
        double targetx = tilex * tileSize[0];
        double targety = tiley * tileSize[1];
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        controller = new Controller(Kp, Ki, Kd, Kf);



        while (opModeIsActive()) {
            Odomentry();
            double yaw   =  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double Vx = controller.Calculate(WrapRads((targetx - Posx)*-1));
            double Vy = controller.Calculate(WrapRads(targety - Posy));

            double x2    =  (Math.cos(yaw) * Vx) - (Math.sin(yaw) * Vy);
            double y2    =  (Math.sin(yaw) * Vx) + (Math.cos(yaw) * Vy);

            double r =  pidR.Calculate(WrapRads(setpoint - yaw));
            double d = Math.max(Math.abs(Vx) + Math.abs(Vy), 1);
            MovePower((y2 - x2 - r) / d, (y2 + x2 + r) / d,
                    (y2 + x2 - r) / d, (y2 - x2 + r) / d);
//            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("XY", "%6f cm %6f cm" , Posx, Posy);
            telemetry.addData("tagetXtargetY", "%6f cm %6f cm" , targetx, targety);
            telemetry.update();
            if (Utilize.AtTargetRange(Posx, targetx, 2) && Utilize.AtTargetRange(Posy, targety, 2)) break;
        }
        Break(0.5);
    }


    public void MovePower(double Front_Left, double Front_Right,
                          double Back_Left,  double Back_Right) {
        FL.setPower(Front_Left );
        FR.setPower(Front_Right);
        BL.setPower(Back_Left);
        BR.setPower(Back_Right);
    }
    public void Break(double stopSecond) {
        if (stopSecond == 0) return;
        MovePower(0, 0, 0, 0);
        MoveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep((long) (stopSecond * 1000));
    }

    public void MoveMode(DcMotor.RunMode moveMode) {
        FL.setMode(moveMode);
        FR.setMode(moveMode);
        BL.setMode(moveMode);
        BR.setMode(moveMode);
    }
    public void Initialize(DcMotor.RunMode moveMode, double[] DuoServoAng, double[] ServoAng) {
        imu = hardwareMap.get(IMU.class,       "imu");
        FL  = hardwareMap.get(DcMotorEx.class, "Front_Left");    FR  = hardwareMap.get(DcMotorEx.class, "Front_Right");
        BL  = hardwareMap.get(DcMotorEx.class, "Back_Left");     BR  = hardwareMap.get(DcMotorEx.class, "Back_Right");
        encoder1 = FL ;
        encoder2 = FR;
        encoder3 = BL;

        // Initialize IMU
      imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection .RIGHT)));
        // Reverse Servo
        // Set Servo Position
        // setMode Motors
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse Motors
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        // SetBehavior Motors
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // SetPower Motors
        MovePower(0, 0, 0, 0);
    }

}