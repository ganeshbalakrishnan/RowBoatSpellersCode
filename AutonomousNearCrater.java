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



@TeleOp(name="AutonomousNearCrater", group="Pushbot")
public class AutonomousNearCrater extends LinearOpMode {
    RobotConfig  robot = new RobotConfig();
    GamePieceDrop dropper  = new GamePieceDrop();
    public ElapsedTime period = new ElapsedTime();
    LatchingControl_Linear latch = new LatchingControl_Linear();
    Orientation angles;
    //private ElapsedTime runtime = new ElapsedTime();
    /* Declare OpMode members. */
    BNO055IMU imu;
    //HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

    @Override
    public void runOpMode()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        robot.init(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!imu.isGyroCalibrated())
        {
            sleep(50);
        }

        waitForStart();
        double power = 0.2;
        double turnAngle = 0;


        //CODE GOES HERE
        moveStringArm(robot,7000,period);
        latch.UnLatchServo(robot);
        moveCm(robot,-60,0.6);
        turnDegrees(robot, -90);
        moveCm(robot,-90,0.6);
        turnDegrees(robot,-45);
        moveCm(robot,-180,0.6);
        dropper.dropGamePiece(robot);
        dropper.dropGamePiece(robot);
    }

    public void turnDegrees(RobotConfig robot, double angle)
    {
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double power = 0.2;
        double accumulatedTurn = 0;
        double startAngle = this.getAbsoluteHeading();
        telemetry.addData("startAngle", startAngle);
        telemetry.update();
        //sleep(3000);
        while(opModeIsActive() && Math.abs(accumulatedTurn) <= Math.abs(angle))
        {
            accumulatedTurn = this.getAbsoluteHeading() - startAngle;
            if (accumulatedTurn < -180)
                accumulatedTurn += 360;
            else if (accumulatedTurn > 180)
                accumulatedTurn -=360;
            if (angle <0){
                robot.leftDrive.setPower(power);
                robot.rightDrive.setPower(-power);
            }
            else{
                robot.leftDrive.setPower(-power);
                robot.rightDrive.setPower(power);
            }
        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    public void moveCm(RobotConfig robot, int dist, double speed)
    {
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int degreesPerCm = 1120/31;
        int error = degreesPerCm;
        int degreesToGo = (degreesPerCm * dist) - error;

        robot.rightDrive.setPower(speed);
        robot.leftDrive.setPower(speed);

        robot.rightDrive.setTargetPosition(degreesToGo);
        robot.leftDrive.setTargetPosition(degreesToGo);

        while (robot.leftDrive.isBusy() && robot.rightDrive.isBusy()){
            telemetry.update();
        }

        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);
    }
    public double getAbsoluteHeading(){
        return this.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
    }
    public void moveStringArm(RobotConfig robotclass, long periodMS, ElapsedTime period)
    {
        period.reset();
        telemetry.addData("function", "has run");
        telemetry.update();
        while (opModeIsActive() && period.milliseconds() < periodMS){

            telemetry.addData("time left", period.milliseconds());
            telemetry.update();
            robotclass.stringArm.setPower(-0.7);//-0.7
        }
        robotclass.stringArm.setPower(0);
    }
}
