package org.firstinspires.ftc.teamcode.opModes.teleop;
import org.firstinspires.ftc.teamcode.subsystems.shooter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "turretTest")

public class turretTest extends LinearOpMode {
    shooter shooter = new shooter();
    double previousAngle = -400;
private Servo rampServo,latchServo;
    private CRServo turretServo;
    double currentAngle = 0;
    private PIDController controller;
    double offset = 0;
    public static double target = 150;
    public static double p=0.006;
    public static double i=0.09;
    public static double d=0.00016;
    private AnalogInput turretEncoder;
    private DcMotorEx shooter1,shooter2,intake;



    public static double speed = 0;
    public static double rampPos = 0;
    public int flywheelSpeed = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretEncoder = hardwareMap.analogInput.get("turretEncoder");
        turretServo = hardwareMap.crservo.get("turretServo");

        shooter1 = (DcMotorEx) hardwareMap.dcMotor.get("shooter1");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2 = (DcMotorEx) hardwareMap.dcMotor.get("shooter2");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        latchServo = hardwareMap.servo.get("latchServo");

        rampServo = hardwareMap.servo.get("rampServo");


        waitForStart();
previousAngle = turretEncoder.getVoltage() / 3.3 * 360;
currentAngle = turretEncoder.getVoltage() / 3.3 * 360;
        while (opModeIsActive()) {
            rampServo.setPosition(rampPos);

            flywheelSpeed = shooter1.getCurrentPosition();
            intake.setPower(1);
            controller.setPID(p,i,d);

            previousAngle = currentAngle;
            currentAngle = ((turretEncoder.getVoltage() / 3.3) * 360) + offset;
            if (currentAngle + 200 < previousAngle) {
                offset += (360);
                currentAngle = ((turretEncoder.getVoltage() / 3.3) * 360) + offset;
            }
            if (currentAngle - 200 > previousAngle) {
                offset -= (360);
                currentAngle = ((turretEncoder.getVoltage() / 3.3) * 360) + offset;
            }

            turretServo.setPower(controller.calculate(Math.round(currentAngle),target));



            shooter1.setVelocity(speed);
            shooter2.setVelocity(speed);
            latchServo.setPosition(gamepad1.left_bumper?.49:.26);
            telemetry.addData("direct read",turretEncoder.getVoltage() / 3.3 * 360);
            telemetry.addData("current angle", currentAngle);

            telemetry.addData("speed", flywheelSpeed);
            telemetry.addData("offset",offset);
            telemetry.addData("target",target);
            telemetry.addData("latch",latchServo.getPosition());

            telemetry.addData("ramp",rampServo.getPosition());

            telemetry.addData("Shooter1 TPS", shooter1.getVelocity());
            telemetry.addData("Shooter2 TPS", shooter2.getVelocity());


            telemetry.addData("Shooter1 pos", shooter1.getCurrentPosition());
            telemetry.addData("Shooter2 pos", shooter2.getCurrentPosition());
            telemetry.update();
            }

        }
    }

