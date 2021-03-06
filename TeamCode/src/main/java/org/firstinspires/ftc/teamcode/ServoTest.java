package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="ServoTest", group="Auto")
public class ServoTest extends LinearOpMode {
    private Servo shooterServo;
    private DcMotor shooter;

    @Override
    public void runOpMode() {
        shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterServo = hardwareMap.servo.get("shooterServo");
        shooterServo.scaleRange(-1,1);
        shooterServo.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        shooter.setPower(1);
        shooterServo.setPosition(0);
        sleep(2000);
        shooter.setPower(1);
        shooterServo.setPosition(.25);
        sleep(2000);
        shooter.setPower(1);
        shooterServo.setPosition(0);
        sleep(2000);
        shooter.setPower(1);
        shooterServo.setPosition(.25);
        sleep(2000);
        shooter.setPower(1);
        shooterServo.setPosition(0);
        sleep(2000);
        shooter.setPower(1);
        shooterServo.setPosition(.25);
        sleep(2000);
    }
}