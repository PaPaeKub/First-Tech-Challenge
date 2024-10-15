package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilize.WrapRads;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Tele")
public class Tele extends Robot {

    private Controller controller;

    // Variables
    int targetLift = 0;
    double setpoint = 0, H_Ang = 0, AL_Ang = 0, AD_Ang = 0;
    boolean autoLift = false, V_Pressed = false, VisBusy = false, ITisOn = false, tp_Pressed = false,
            H_disable = false, r_disable = false;

    private void Init() {
        // Initialize Robot
        Initialize(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new double[]{0, 0, AL_Ang},
                                                        new double[]{0, 0, 0, 0});

        // Show FTC Dashboard
        controller = new Controller(0.9, 0.01, 0.008, 0);
    }

    private void Movement() {
        double speed = 0.275;
        double lx = -gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double x1 = gamepad1.dpad_left ? speed : gamepad1.dpad_right ? -speed : lx;
        double y1 = gamepad1.dpad_up ? -speed : gamepad1.dpad_down ? speed : ly;
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double x2 = (Math.cos(yaw) * x1) - (Math.sin(yaw) * y1);
        double y2 = (Math.sin(yaw) * x1) + (Math.cos(yaw) * y1);
        // Rotate
        double r = r_disable ? 0 : controller.Calculate(WrapRads(setpoint - yaw));
        double x = -gamepad1.right_stick_x;
        if (x != 0) {
            r = x;
            setpoint = yaw;
        }
        // Denominator for division to get no more than 1
        double d = Math.max(Math.abs(x2) + Math.abs(y2) + Math.abs(r), 1);
        MovePower((y2 - x2 - r) / d, (y2 + x2 + r) / d,
                (y2 + x2 - r) / d, (y2 - x2 + r) / d);
        telemetry.addData("yaw", Math.toDegrees(-yaw));
    }
    private void Lift (){
        CurPosLift    = Math.max(LL.getCurrentPosition(), RL.getCurrentPosition());
        double lt            = gamepad2.left_trigger;
        double rl            = gamepad2.right_trigger;
        boolean ltpress      = lt >= 0.25;
        boolean rlpress      = rl >= 0.25;
        Lift_Power   = ltpress ? (CurPosLift < -30000          ?  0   : -lt) :
                       rlpress ? (CurPosLift > 30000       ?  0   :  rl) : 0;
        liftPower(Lift_Power);
    }
    private void ARM (){
        if(gamepad1.x)  SetServoPos(0.55, RA, LA);
        if(gamepad1.a)  SetServoPos(0.0, RA, LA);
    }

    @Override
    public void runOpMode() {
        Init();
        RA.setPosition(0);
        LA.setPosition(0);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Lift();
                Odomentry();
                ARM();

                telemetry.addData("XYH", "%6f cm %6f cm %6f deg", Posx, Posy, Math.toDegrees(heading));
                telemetry.addData("LRM", "%6d  %6d %6d", Current1Position, Current2Position, Current3Position);
                telemetry.addData("Lift", "%6f", CurPosLift);
                telemetry.addData("Liftpower", "%6f", Lift_Power);
                telemetry.update();
                if (gamepad1.y) {
                    LL.setPower(1);
                    RL.setPower(1);
                }
                if (gamepad1.b) {
                    n += 0.01 ;
                    n = Math.min(n,0.55);
                    RA.setPosition(n);
                    LA.setPosition(n);

                }
            }
        }
    }
}

