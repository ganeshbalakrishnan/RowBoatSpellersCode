package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotConfig;



/**e
 * Created by manav on 09/30/18.
 */
@TeleOp(name="HandMovement", group="Pushbot")
public class HandMovement extends LinearOpMode
{
    RobotConfig  robot = new RobotConfig();

    Orientation angles;

    public float clawSpeed;
    public void runOpMode()
    {

        robot.init(hardwareMap);




        waitForStart();
        double power = 0.2;
        double turnAngle = 0;
        // the while loop below actually runs the code
        /*while (opModeIsActive()){
            robot.leftArm.setPower(-0.3);
            robot.WaitForTick(1000);
            robot.leftArm.setPower(0);

        }
        */
        robot.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //getData();
        handTurnDegrees(135);
        //elbowTurn(90);

        while (opModeIsActive()){

        }

    }
    public void getData(){
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()){
            robot.leftArm.setPower(0.5);
            telemetry.addData("position", robot.leftArm.getCurrentPosition());
            telemetry.update();
        }
    }
    public void getDataTwo(){
        while (opModeIsActive()){
            robot.elbow.setPosition(90);
            telemetry.addData("position", robot.elbow.getPosition());
            telemetry.update();
        }
    }
    public void handTurnDegrees(int angle)
    {
        /*
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //int ArmPosition = robot.leftArm.getCurrentPosition();
        double power = 0.1;
        //double accumulatedTurn = 0;
        //telemetry.addData("startAngle", robot.leftArm.getCurrentPosition());
        //telemetry.update();
        //sleep(100);
        robot.leftArm.setTargetPosition(angle*3);
        robot.leftArm.setPower(power);
        /*
        while(opModeIsActive() && robot.leftArm.getCurrentPosition() < angle){
            //telemetry.addData("currentAngle", robot.leftArm.getCurrentPosition());
            //telemetry.update();
            if (angle > 0){
                robot.leftArm.setPower(power);
            }
            else if (angle < 0){
                robot.leftArm.setPower(-power);
            }
            telemetry.addData("angle", robot.leftArm.getCurrentPosition());
            telemetry.update();
        }
        */
        //robot.leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.leftArm.setPower(0);
    }
    public void elbowTurn(double angle)
    {
        telemetry.addData("Angle",robot.elbow.getPosition());
        telemetry.update();
        sleep(10);
        if (angle < robot.elbow.getPosition()){
            robot.elbow.setDirection(Servo.Direction.FORWARD);
        }
        else if (angle > robot.elbow.getPosition())
        {
            robot.elbow.setDirection(Servo.Direction.REVERSE);
        }

        robot.elbow.setPosition(angle);
    }

}
