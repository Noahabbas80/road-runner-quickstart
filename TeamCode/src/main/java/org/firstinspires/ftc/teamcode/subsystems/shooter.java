package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class shooter {
    private PIDController controller;
    private Servo latchServo,rampServo;
    private AnalogInput turretEncoder;
    public IMU imu;
    public double previousAngle, currentAngle = 180;
    public int offset = 0;
    public static double p,i,d = 0;
    private DcMotorEx shooter1,shooter2;
    public static double speed = 0;
    private CRServo turretServo;
    public void init(HardwareMap hwm, Telemetry telemetry){
        controller = new PIDController(p,i,d);

        turretEncoder = hwm.analogInput.get("turretEncoder");
        shooter1 = (DcMotorEx) hwm.dcMotor.get("shooter1");
        shooter2 = (DcMotorEx) hwm.dcMotor.get("shooter2");
        turretServo = hwm.crservo.get("turretServo");
        latchServo = hwm.servo.get("latchServo");
        rampServo = hwm.servo.get("rampServo");

        imu = hwm.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu.initialize(new IMU.Parameters(orientation));

        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        latchServo.setPosition(.57);

    }

    public void fire(){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < 1.5) {
            latchServo.setPosition(.57);
        }
            latchServo.setPosition(.05);
        }

        public void manual(Gamepad gamepad){
            turretServo.setPower(gamepad.left_trigger + -gamepad.right_trigger);
        }


        public void start(double power){
            shooter1.setVelocity(speed);
            shooter1.setVelocity(speed);
        }
        public double getRelativeAngle() {
            previousAngle = currentAngle;
            currentAngle = turretEncoder.getVoltage() / 3.3 * 360 + offset;
            if (currentAngle + 200 < previousAngle) {
                offset += 360;
                currentAngle = turretEncoder.getVoltage() / 3.3 * 360 + offset;
            }
            if (currentAngle - 200 > previousAngle) {
                offset -= 360;
                currentAngle = turretEncoder.getVoltage() / 3.3 * 360 + offset;
            }
            return currentAngle;
        }



    public void align(double offset[],Telemetry telemetry){
        //make separate pid values because u are sped
        controller.setPID(p,i,d);
//        turretServo.setPower((Double.isNaN(offset[0]) ? controller.calculate(currentAngle, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) : controller.calculate(offset,0)));
        if(Double.isNaN(offset[0])){
            turretServo.setPower(controller.calculate(offset[0],0));
        }
        else{
//            turretServo.setPower(controller.calculate(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),180));
            turretServo.setPower(0);
        }
//
        telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));

        telemetry.update();

    }
}


