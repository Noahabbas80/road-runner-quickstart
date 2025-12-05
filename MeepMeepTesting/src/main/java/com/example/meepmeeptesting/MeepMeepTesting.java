package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        Pose2d startPose = new Pose2d(-0 , -0, Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15.3543)
                .build();
        Vector2d botOffset = new Vector2d(16.7717/2,15.3543/2);
        Pose2d startPos = new Pose2d(-24-botOffset.x,-24+botOffset.y,Math.toRadians(270));
        Vector2d firePos = new Vector2d(-24-botOffset.x,-24+botOffset.y);

        myBot.runAction(myBot.getDrive().actionBuilder(startPos)
                .strafeTo(new Vector2d(-12,-24+botOffset.y))
                .strafeTo(new Vector2d(-12,-65+botOffset.y))
                .setTangent(45)
                .splineToConstantHeading(new Vector2d(-2,-55), Math.toRadians(270))
                .setTangent(90)
                .splineToConstantHeading(firePos, Math.toRadians(135))
                .strafeTo(new Vector2d(12,-24+botOffset.y))
                .strafeTo(new Vector2d(12,-65+botOffset.y))
                .setTangent(90)
                .splineToConstantHeading(firePos, Math.toRadians(135))
                .strafeTo(new Vector2d(36,-24+botOffset.y))
                .strafeTo(new Vector2d(36,-65+botOffset.y))
                .setTangent(90)
                .splineToConstantHeading(firePos, Math.toRadians(135))
                .strafeTo(new Vector2d(-2,-40))
//                .strafeTo(new Vector2d(36,-27+botOffset.y))
//                .strafeTo(new Vector2d(36,-72+botOffset.y))
//                .setTangent(45)
//                .splineToConstantHeading(firePos, Math.toRadians(45))
//                .strafeTo(new Vector2d(-12,-27+botOffset.y))
//                .strafeTo(new Vector2d(-12,-65+botOffset.y))
//                .setTangent(45)
//                .splineToConstantHeading(firePos, Math.toRadians(0))
//                .strafeTo(new Vector2d(69-botOffset.x,-50))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}