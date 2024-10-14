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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;

public abstract class Robot extends LinearOpMode {
    public IMU imu;
    public VisionPortal visionPortal;
    public Servo LA, RA, LH, RH, ALL, ARL, IT, DP, ADP;
    public DcMotorEx FL, FR, BL, BR, encoder1, encoder2, encoder3 ;
    public int FL_Target, FR_Target, BL_Target, BR_Target;
    public final double[] tileSize            = {60.96, 60.96};  // Width * Length
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
    public final double   L                   = 32.50; //distance between 1 and 2 in cm
    public final double   B                   = 19.0; //distance between center of 1 and 2 and 3 in cm
    public final double   r                   = 2.5 ; // Odomentry wheel radius in cm
    public final double   N                   = 2000 ; // ticks per one rotation
    public double         cm_per_tick         = 2.0 * Math.PI * r / N ;
    public int            dn1, dn2, dn3 ;
    public double         dx, dy, Posx, Posy, heading  ;

    // update encoder
    public int            Current1Position= 0 ;
    public int            Current2Position= 0 ;
    public int            Current3Position= 0 ;

    public int            Old1Position= 0 ;
    public int            Old2Position= 0 ;
    public int            Old3Position= 0;

    public void Odomentry (){

        double yaw   =  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
      Old1Position         = Current1Position;
      Old2Position         = Current2Position;
      Old3Position         = Current3Position;

      Current1Position     = encoder1.getCurrentPosition();
      Current2Position     = encoder2.getCurrentPosition();
      Current3Position     = -encoder3.getCurrentPosition();

      dn1 = Current1Position - Old1Position;
      dn2 = Current2Position - Old2Position;
      dn3 = Current3Position - Old3Position;

      dy = cm_per_tick * ( dn1 + dn2 ) / 2.0 ;
      dx = cm_per_tick * ( dn3 - ( dn2 - dn1 ) * B / L );
      double dthetha = cm_per_tick * ( dn2 - dn1 ) / L ;

      double theta = heading + (dn2 - dn1) / L;
      Posy -= dy * Math.cos(yaw) - dx * Math.sin(yaw);
      Posx += dy * Math.sin(yaw) + dx * Math.cos(yaw);
      heading += dthetha;
    }

    public void move(double tilex, double tiley, double timeout , double setpoint){
        Controller  pidR    = new Controller(1.0, 0.01, 0.04, 0);
        double targetx = tilex * tileSize[0];
        double targety = tiley * tileSize[1];
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();


        while (opModeIsActive()) {
            Odomentry();
            double yaw   =  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double DeltaX = targetx - Posx;
            double DeltaY = targety - Posy;

            double Vx = -0.1 * DeltaX;
            double Vy = 0.1 * DeltaY;

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
            if (Utilize.AtTargetRange(Posx, targetx, 10) && Utilize.AtTargetRange(Posy, targety, 10)) break;
        }
        Break(0.3);
    }


    public void MovePower(double Front_Left, double Front_Right,
                          double Back_Left,  double Back_Right) {
        FL.setPower(Front_Left);
        FR.setPower(Front_Right);
        BL.setPower(Back_Left*0.9);
        BR.setPower(Back_Right*0.9);
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
                RevHubOrientationOnRobot.UsbFacingDirection .LEFT)));

        // Reverse Servo

        // Set Servo Position

        // setMode Motors
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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