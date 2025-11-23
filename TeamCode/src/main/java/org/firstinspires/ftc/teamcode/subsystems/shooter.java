package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class shooter {
    private PIDController controller;
    public static double p,i,d,target = 0;
    private DcMotor shooter1,shooter2;
    private CRServo turretServo;
    public void init(HardwareMap hwm, Telemetry telemetry){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter1 = hwm.dcMotor.get("shooter1");
        shooter2 = hwm.dcMotor.get("shooter2");
        turretServo = hwm.crservo.get("turretServo");

        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void align(int offset){
        controller.setPID(p,i,d);
        //might need to fix / reevalute this since we only have offset
        double pid = controller.calculate(offset,0);
        turretServo.setPower(pid);
    }

    public void fire(double power){
        shooter1.setPower(power);
        shooter2.setPower(power);

    }
}
