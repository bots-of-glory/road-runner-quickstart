package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "DriverControl" , group = "testOp")
//@Disabled

public class DriverControl extends LinearOpMode {
    //DcMotors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor rearLeft;
    private DcMotor frontRight;
    private DcMotor rearRight;
    private Servo shooterServo;
    private CRServo claw;
    private DcMotor clawLift;
    private DcMotor shooter;
    private DcMotor intake;



    @Override
    public void runOpMode () throws InterruptedException {
        //Declare DcMotors
        shooter = hardwareMap.dcMotor.get("shooter");
        shooterServo = hardwareMap.servo.get("shooterServo");
        claw = hardwareMap.crservo.get("claw");
        clawLift = hardwareMap.dcMotor.get("clawLift");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearRight = hardwareMap.dcMotor.get("rearRight");
        intake = hardwareMap.dcMotor.get("intake");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        clawLift.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        clawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Declare Mecanum Drive Variables
        double drive;
        double strafe;
        double rotate;

        double front_left;
        double rear_left;
        double front_right;
        double rear_right;

        //Declare Speed Variables(0 = slow)(1 = fast)
        int speedState = 1;
        double fast = 1;

        float   leftPower, rightPower, xValue, yValue;
        //Declare Direction Variable(s)
        int direction = -1;
        {
            waitForStart();

            while (opModeIsActive()) {
//-----------------------------------Gamepad 1 Start------------------------------------------------

                //Declare Values to Mecanum Variables
                drive = gamepad1.right_stick_y * direction;
                strafe = gamepad1.right_stick_x * direction;
                rotate = gamepad1.left_stick_x * direction;

                //Mecanum direction calculation

                    front_left = drive - strafe - rotate;
                    rear_left = drive + strafe - rotate;
                    front_right = drive + strafe + rotate;
                    rear_right = drive - strafe + rotate;


                frontLeft.setPower((front_left) * fast);
                rearLeft.setPower((rear_left) * fast);
                frontRight.setPower((front_right) * fast);
                rearRight.setPower((rear_right) * fast);


                //---------claw----------
                while (gamepad1.dpad_down) {
                    clawLift.setPower(1);
                }
                while (gamepad1.dpad_up) {
                    clawLift.setPower(-1);
                }

                if (gamepad1.dpad_left) {
                    claw.setPower(1);
                } else if (gamepad1.dpad_right) {
                    claw.setPower(-1);
                }

//------------------------------------Gamepad 1 End-------------------------------------------------
// ------------------------------------Gamepad 2 Start-------------------------------------------------
                if (gamepad2.right_trigger == 1) {
                    shooterServo.setPosition(-1);
                } else {
                    shooterServo.setPosition(1);
                }

                if (gamepad2.a) {
                    shooter.setPower(1);
                }

                if (gamepad2.left_bumper) {
                    intake.setPower(1);
                } else if (gamepad2.right_bumper) {
                    intake.setPower(-1);
                }




            }
//------------------------------------Gamepad 2 End-------------------------------------------------


            idle();
        }
    }
    public double limit(double number)
    {
        if(number < -1.0)
        {
            return -1.0;
        }
        else if(number > 1)
        {
            return 1;
        }
        else
        {
            return number;
        }
    }
}
