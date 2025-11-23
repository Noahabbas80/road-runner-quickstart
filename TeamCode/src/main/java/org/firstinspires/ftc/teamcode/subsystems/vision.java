package org.firstinspires.ftc.teamcode.subsystems;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

public class vision  {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public void init(HardwareMap hwm){
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(69, 69, 69, 69)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwm.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(1280, 800));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();


    }

}
