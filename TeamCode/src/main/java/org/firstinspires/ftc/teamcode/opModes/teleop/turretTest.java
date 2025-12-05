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
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "turretTest")

public class turretTest extends LinearOpMode {
    shooter shooter = new shooter();
    double previousAngle = -400;
private Servo rampServo;
    private CRServo turretServo;
    double currentAngle = 0;
    double offset = 0;
    private AnalogInput turretEncoder;
    private DcMotorEx shooter1,shooter2,intake;
    private PIDController controller;

    public static double target = 150;
    public static double p=0.006;
    public static double i=0.09;
    public static double d=0.00016;
    public static double speed = 0;
    public int flywheelSpeed = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretEncoder = hardwareMap.analogInput.get("turretEncoder");

        shooter1 = (DcMotorEx) hardwareMap.dcMotor.get("shooter1");
        turretServo = hardwareMap.crservo.get("turretServo");

        rampServo = hardwareMap.servo.get("rampServo");
        shooter2 = (DcMotorEx) hardwareMap.dcMotor.get("shooter2");

        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");

        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
previousAngle = turretEncoder.getVoltage() / 3.3 * 360;
currentAngle = turretEncoder.getVoltage() / 3.3 * 360;
        while (opModeIsActive()) {
            rampServo.setPosition(.2 + gamepad1.right_trigger);
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

            shooter1.setVelocity(speed);
            shooter2.setVelocity(speed);

            turretServo.setPower(gamepad1.x ? 1 : controller.calculate(Math.round(currentAngle),target));

            telemetry.addData("direct read",turretEncoder.getVoltage() / 3.3 * 360);
//            telemetry.addData("previous angle", previousAngle);
            telemetry.addData("current angle", currentAngle);
            telemetry.addData("speed", flywheelSpeed);
            telemetry.addData("offset",offset);
            telemetry.addData("target",target);
            telemetry.update();
            }

        }
    }

