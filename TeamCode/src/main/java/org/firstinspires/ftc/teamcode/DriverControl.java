package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "DriverControl" , group = "testOp")

public class DriverControl extends LinearOpMode {
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
        shooterServo.scaleRange(-1,1);
        shooterServo.setDirection(Servo.Direction.REVERSE);

        //Declare Mecanum Drive Variables
        double drive;
        double strafe;
        double rotate;

        double front_left;
        double rear_left;
        double front_right;
        double rear_right;

        //Declare Speed Variables(0 = slow)(1 = fast)
        double fast = 1;
        double slow = .5;
        int speedstate = 1;
        //Declare Direction Variable(s)
        int direction = -1;
        {
            waitForStart();

            while (opModeIsActive()) {
//-----------------------------------Gamepad 1 Start------------------------------------------------

                if (gamepad1.left_bumper) {
                    speedstate = 0;
                } else if (gamepad1.right_bumper) {
                    speedstate = 1;
                }
                //Declare Values to Mecanum Variables
                drive = gamepad1.right_stick_y * direction;
                strafe = gamepad1.right_stick_x * direction;
                rotate = gamepad1.left_stick_x * direction;

                //Mecanum direction calculation

                front_left = drive - strafe - rotate;
                rear_left = drive + strafe - rotate;
                front_right = drive + strafe + rotate;
                rear_right = drive - strafe + rotate;
                    if (speedstate == 1) {
                        frontLeft.setPower((front_left) * fast);
                        rearLeft.setPower((rear_left) * fast);
                        frontRight.setPower((front_right) * fast);
                        rearRight.setPower((rear_right) * fast);
                    }
                    else {
                        frontLeft.setPower((front_left) * slow);
                        rearLeft.setPower((rear_left) * slow);
                        frontRight.setPower((front_right) * slow);
                        rearRight.setPower((rear_right) * slow);
                    }
                    //---------claw----------
                    while (gamepad1.dpad_down) {
                        clawLift.setPower(1);
                    }
                    while (gamepad1.dpad_up) {
                        clawLift.setPower(-1);
                    }
                        clawLift.setPower(0);

                    while (gamepad1.dpad_left) {
                        claw.setPower(-1);
                    }
                    while (gamepad1.dpad_right) {
                        claw.setPower(1);
                    }

//------------------------------------Gamepad 1 End-------------------------------------------------
// ------------------------------------Gamepad 2 Start-------------------------------------------------
                    if (gamepad2.right_trigger == 1) {
                        shooterServo.setPosition(.25);
                    } else {
                        shooterServo.setPosition(-1);
                    }

                    if (gamepad2.a) {
                        shooter.setPower(1);
                    }

                    if (gamepad2.left_bumper) {
                        intake.setPower(1);
                    } else if (gamepad2.right_bumper) {
                        intake.setPower(-1);
                    }
                    if (gamepad2.dpad_down) {
                        shooter.setPower(0);
                        intake.setPower(0);
                    }
            }
//------------------------------------Gamepad 2 End-------------------------------------------------


            idle();
        }
    }
}
