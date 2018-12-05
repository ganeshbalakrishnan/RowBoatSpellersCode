package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotConfig;

@TeleOp (name= "DistanceSensor", group = "Pushbot")
public class DistanceSensorClass extends LinearOpMode
{
    DistanceSensor distSensorFront;
    DistanceSensor distSensorLeft;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    double power = 0.3;
    @Override

    public void runOpMode()
    {
        distSensorFront = hardwareMap.get(DistanceSensor.class, "front_distance_sensor");
        distSensorLeft = hardwareMap.get(DistanceSensor.class, "left_distance_sensor");
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        Rev2mDistanceSensor sensorTimeofFlight = (Rev2mDistanceSensor)distSensorFront;
        Rev2mDistanceSensor sensorTimeofFlightTwo = (Rev2mDistanceSensor)distSensorLeft;
        telemetry.addData(">>", "Press Start to Continue");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && sensorTimeofFlight.getDistance(DistanceUnit.CM) > 10)
        {
            //telemetry.addData("deviceCame",distSensorLeft.getDeviceName());
            //telemetry.addData("range",String.format("%.01cm",distSensorLeft.getDistance(DistanceUnit.CM)>10));
            //telemetry.addData("ID",String.format("%x", sensorTimeofFlightTwo.getModelID()));
            // telemetry.addData("did time out", Boolean.toString(sensorTimeofFlightTwo.didTimeoutOccur()));
            //telemetry.update();

            if (sensorTimeofFlightTwo.getDistance(DistanceUnit.CM) > 12){
                telemetry.addData("sensor", "mustMove");
                telemetry.addData("range",String.format("%.01f",distSensorLeft.getDistance(DistanceUnit.CM)));
                telemetry.update();
                leftMotor.setPower(-0.1);
                rightMotor.setPower(-0.2);
            }
            if (sensorTimeofFlightTwo.getDistance(DistanceUnit.CM) < 11){
                telemetry.addData("sensor", "mustNotMove");
                telemetry.update();
                leftMotor.setPower(-0.2);
                rightMotor.setPower(-0.1);
            }
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
