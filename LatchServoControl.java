package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.RobotConfigNameable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by manav on 11/25/18.
 */
@TeleOp(name="LatchServoControl", group="Pushbot")
public class LatchServoControl extends LinearOpMode{
    RobotConfig robot = new RobotConfig();
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        robot.gamePieceServo.setPosition(10);
        for (int i = 0;i <1; i ++){
            robot.gamePieceServo.setPosition(robot.gamePieceServo.getPosition()-1);
            sleep(500);
        }
    }
}
