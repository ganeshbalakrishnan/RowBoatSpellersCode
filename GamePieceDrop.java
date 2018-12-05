package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.RobotConfigNameable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="gamePieceDrop", group="Pushbot")
public class GamePieceDrop extends LinearOpMode {
    RobotConfig robot = new RobotConfig();

    double clawOffset = 2;
    double clawPos = 0.7;
    double CLAW_SPEED = 0.02;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        //dropGamePiece();
        //robot.gamePieceServo.setPosition(10);
        //robot.gamePieceServo.setPosition(clawPos);
        // robot.gamePieceServo.setPosition(0.5);
        //dropGamePiece();

        // for (int i = 0;i <1; i ++){
        //       robot.gamePieceServo.setPosition(robot.gamePieceServo.getPosition()-1);
        //       sleep(600);
        //  }
        // for (int i = 0;i <1; i ++){
        //       robot.gamePieceServo.setPosition(robot.gamePieceServo.getPosition()+1);
        //       sleep(600);
        //  }
        //  robot.gamePieceServo.setPosition(0.5);
    }


    public void dropGamePiece(RobotConfig robot){
        for (int i = 0;i <1; i ++){
            robot.gamePieceServo.setPosition(robot.gamePieceServo.getPosition()-1);
            sleep(1000);
        }
        //robot.gamePieceServo.setPosition(0.5);
        sleep(1000);
        for (int i = 0;i <1; i ++){
            robot.gamePieceServo.setPosition(robot.gamePieceServo.getPosition()+1);
            sleep(600);
        }
        robot.gamePieceServo.setPosition(0.5);
    }

}