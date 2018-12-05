
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotConfig
{
    /* Public OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor stringArm = null;
    public Servo   elbow    = null;
    public Servo   claw   = null;
    public Servo gamePieceServo = null;
    public Servo hookServo = null;

    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    //public HardwarePushbot(){

    //}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotor.class, "left_motor");
        rightDrive = hwMap.get(DcMotor.class, "right_motor");
        stringArm = hwMap.get(DcMotor.class, "leftArm");

        //actually makes the robot go forward ()
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        stringArm.setPower(0);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stringArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        //elbow  = hwMap.get(Servo.class, "stringArm");
        //eclaw = hwMap.get(Servo.class, "claw");
        gamePieceServo = hwMap.get(Servo.class, "gamePieceServo");
        hookServo = hwMap.get(Servo.class, "hookServo");
        //rightClaw.setPosition(MID_SERVO);
    }

    public void WaitForTick(long periodMS)
    {
        long remaining = periodMS - (long) period.milliseconds();
        if (remaining > 0)
        {
            try
            {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        period.reset();
    }
}

