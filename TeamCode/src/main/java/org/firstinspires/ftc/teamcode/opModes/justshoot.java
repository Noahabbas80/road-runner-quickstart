package org.firstinspires.ftc.teamcode.opModes;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
@TeleOp(name = "justshoot")

public class justshoot extends LinearOpMode {
    DcMotor rightFront, rightBack, leftFront, leftBack;
    Servo latchServo,turretServo,rampServo;

    public boolean okey = false;
    boolean ended = false;
    ElapsedTime runtime = new ElapsedTime();
    public Gamepad currentGamepad1 = new Gamepad();
    public static int position = 0;
    public Gamepad previousGamepad1 = new Gamepad();
    vision vision = new vision();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int swordID = hardwareMap.appContext.getResources().getIdentifier("sword", "raw", hardwareMap.appContext.getPackageName());
        vision.init(hardwareMap);
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        latchServo = hardwareMap.servo.get("latchServo");

        rampServo = hardwareMap.servo.get("rampServo");
        turretServo = hardwareMap.servo.get("turretServo");

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


latchServo.setPosition(.5);
        turretServo.setPosition(.5);
        rampServo.setPosition(1);
        waitForStart();

        runtime.reset();
        while (opModeIsActive()) {
            vision.offset(telemetry,21);
            position = leftFront.getCurrentPosition();
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (runtime.seconds() > 5 && !ended)
            {
                ended = true;
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, swordID);
            }

            leftFront.setPower(gamepad1.cross ? 1 : 0);
            leftBack.setPower(gamepad1.cross ? 1 : 0);
            rightFront.setPower(gamepad1.cross ? 1 : 0);
            rightBack.setPower(gamepad1.cross ? 1 : 0);
            if(currentGamepad1.square && !previousGamepad1.square){
                okey = !okey;
            }
            latchServo.setPosition(okey ? .75 : .507);
//            rampServo.setPosition(gamepad1.right_stick_y/2 + .5);
            turretServo.setPosition(gamepad1.left_stick_x/2 + .5);
            telemetry.addData("pos",-1*leftBack.getCurrentPosition());
            telemetry.addData("ramp pos", rampServo.getPosition());

            telemetry.addData("latch pos", latchServo.getPosition());

            telemetry.addData("turret pos", turretServo.getPosition());

            telemetry.update();
            }

        }
    }

