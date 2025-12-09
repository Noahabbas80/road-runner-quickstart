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
    private Servo latchServo,rampServo,turretServo;
    private DcMotorEx shooter1,shooter2;
    public static double speed = 1400;
    public void init(HardwareMap hwm, boolean isBlue){
        shooter1 = (DcMotorEx) hwm.dcMotor.get("shooter1");
        shooter2 = (DcMotorEx) hwm.dcMotor.get("shooter2");

        turretServo = hwm.servo.get("turretServo");
        latchServo = hwm.servo.get("latchServo");
        rampServo = hwm.servo.get("rampServo");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        latchServo.setPosition(.26);
        rampServo.setPosition(.01);
        shooter1.setDirection(DcMotor.Direction.REVERSE);
        shooter1.setVelocity(1400);
        shooter2.setVelocity(1400);




    }

    public void fire(){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < 1.25) {
            latchServo.setPosition(.49);
        }
            latchServo.setPosition(.26);
        }

        public void setRPM(double power){
            shooter1.setVelocity(speed);
            shooter2.setVelocity(speed);
        }


        public void align(double tx){
            if(tx > .5){
                turretServo.setPosition(turretServo.getPosition() + tx * (140/64));
            }

        }
}


