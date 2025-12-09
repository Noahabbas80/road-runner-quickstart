package org.firstinspires.ftc.teamcode.trash;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "justshoot")

public class justshoot extends LinearOpMode {
    DcMotor shooter1, shooter2, intake;
    CRServo turretServo;
    Servo latchServo,rampServo;

    public boolean okey = false;
    boolean ended = false;
    ElapsedTime runtime = new ElapsedTime();
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int swordID = hardwareMap.appContext.getResources().getIdentifier("sword", "raw", hardwareMap.appContext.getPackageName());

        shooter1 = hardwareMap.dcMotor.get("shooter1");
        shooter2 = hardwareMap.dcMotor.get("shooter2");
        intake = hardwareMap.dcMotor.get("intake");
        turretServo = hardwareMap.crservo.get("turretServo");
        latchServo = hardwareMap.servo.get("latchServo");
        rampServo = hardwareMap.servo.get("rampServo");
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        runtime.reset();
        while (opModeIsActive()) {

            rampServo.setPosition(.2 + gamepad1.right_trigger);
            turretServo.setPower(gamepad1.left_stick_x);
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (runtime.seconds() > 5 && !ended)
            {
                ended = true;
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, swordID);
            }

            shooter1.setPower(gamepad1.cross ? 1 : 0);
            shooter2.setPower(gamepad1.cross ? 1 : 0);
            intake.setPower(gamepad1.left_trigger);
            latchServo.setPosition(gamepad1.right_stick_y*.5 + .5);
            if(currentGamepad1.square && !previousGamepad1.square){
                okey = !okey;
            }
            turretServo.setPower(gamepad1.left_stick_x);

            telemetry.addData("turret pos", turretServo.getPower());
telemetry.addData("e",gamepad1.left_trigger);
telemetry.addData("e",gamepad1.right_stick_y);
telemetry.addData("ramp servo pos",rampServo.getPosition());
            telemetry.update();
            }

        }
    }

