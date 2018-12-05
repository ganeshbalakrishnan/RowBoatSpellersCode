package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import android.graphics.Color;


import com.qualcomm.robotcore.hardware.ColorSensor;
/**
 * Created by manav on 10/18/18.
 */
@TeleOp(name="ColorSensor", group="Pushbot")
public class ColorSensorClass extends LinearOpMode{
    ColorSensor sensorColor;
    AutonomousNearDepot turnClass = new AutonomousNearDepot();
    RobotConfig robot = new RobotConfig();

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        sensorColor = hardwareMap.get(ColorSensor.class, "rightColorSensor");
        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout","id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        waitForStart();


        while (opModeIsActive()){
            Color.RGBToHSV((int) (sensorColor.red()*SCALE_FACTOR),(int) (sensorColor.green()*SCALE_FACTOR),(int) (sensorColor.blue()*SCALE_FACTOR),hsvValues);


            if (values[0] >= 190 && values[0] <= 280){
                telemetry.addData("Color", values[0]);
                telemetry.update();
                for (int i = 0; i < 5; i ++){
                    robot.leftDrive.setPower(-0.5);
                    robot.rightDrive.setPower(0.5);
                    sleep(10);
                }

            }
            else{
                telemetry.addData("Color","not red");

                robot.rightDrive.setPower(-0.5);
                robot.leftDrive.setPower(-0.5);


            }
            telemetry.update();
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff,values));
                }
            });

        }

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });

    }

}
