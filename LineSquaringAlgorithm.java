package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import android.graphics.Color;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotConfig;
import com.qualcomm.robotcore.hardware.ColorSensor;
/**
 * Created by manav on 11/18/18.
 */
@TeleOp(name="LineSquare", group="Pushbot")
public class LineSquaringAlgorithm extends LinearOpMode{
    ColorSensor rightColorSensor;
    ColorSensor leftColorSensor;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    double power = 0.3;
    @Override
    public void runOpMode(){
        rightColorSensor = hardwareMap.get(ColorSensor.class, "rightColorSensor");
        leftColorSensor = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor.setDirection(DcMotor.Direction.REVERSE);


        float rightHsvValues[] = {0F,0F,0F};
        final float rightValues[] = rightHsvValues;
        final double SCALE_FACTOR = 255;
        float leftHsvValues[] = {0F,0F,0F};
        final float leftValues[] = leftHsvValues;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout","id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        
        
        Color.RGBToHSV((int) (rightColorSensor.red()*SCALE_FACTOR),
                (int) (rightColorSensor.green()*SCALE_FACTOR),
                (int) (rightColorSensor.blue()*SCALE_FACTOR),
                rightHsvValues);
        Color.RGBToHSV((int) (leftColorSensor.red()*SCALE_FACTOR),
                (int) (leftColorSensor.green()*SCALE_FACTOR),
                (int) (leftColorSensor.blue()*SCALE_FACTOR), leftHsvValues);

        waitForStart();
        for (int i = 0; i < 3; i ++){
            while (opModeIsActive() && !detectYellow(rightValues) && !detectYellow(leftValues)){
                leftMotor.setPower(power);
                rightMotor.setPower(power);
            }
            if (detectYellow(rightValues) && !detectYellow(leftValues)){ //yellow on right
                leftMotor.setPower(power);
            }
            else if (detectYellow(leftValues) && !detectYellow(rightValues)){ // yellow on left
                rightMotor.setPower(power);
            }
            if (i != 2){
                leftMotor.setPower(-power);
                rightMotor.setPower(-power);
            }
            else
            {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
        }
    }

    public boolean detectYellow(float values[])
    {
        if (values[0] >= 40 && values[0]<= 90)
        {
            return true;
        }
        return false;
    }
}
