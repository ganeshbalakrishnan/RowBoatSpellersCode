package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotConfig;

/**
 * Created by manav on 09/16/18.
 */
@TeleOp (name= "JoystickMovement", group = "Pushbot")
public class JoystickControl extends LinearOpMode
{
    RobotConfig  robot = new RobotConfig();
    LatchingControl_Linear latch = new LatchingControl_Linear();
    @Override
    public void runOpMode()
    {
        double left;
        double right;
        double max;
        double elbowVal;
        double armVal;
        double drive;
        double turn;
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
//            drive = gamepad1.left_stick_y;
//            turn = gamepad1.left_stick_x;
//            robot.leftDrive.setPower(drive);
//            robot.rightDrive.setPower(drive);
//            robot.leftDrive.setPower(turn);
//            robot.rightDrive.setPower(-turn);
//            //left = gamepad1.left_stick_y;
//            //right = gamepad1.right_stick_y;

            drive = gamepad1.left_stick_y;
            turn  =  -gamepad1.left_stick_x;
            left   = Range.clip(drive + turn, -1.0, 1.0) ;
            right   = Range.clip(drive - turn, -1.0, 1.0) ;
            if (gamepad1.dpad_up){
                armVal = 1;
            }
            else if (gamepad1.dpad_down){
                armVal = -1;
            }
            else{
                armVal = 0;
            }
            if (gamepad1.a){
                latch.turnLatchServo(robot);
            }
            else if (gamepad1.b){
                latch.UnLatchServo(robot);
            }
            /*
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0){
                left /= max;
                right /= max;
            }
            */
            robot.stringArm.setPower(armVal/3);
            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);
        }

    }
}
