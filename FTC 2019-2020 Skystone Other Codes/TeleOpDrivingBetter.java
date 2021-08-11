package org.firstinspires.ftc.teamcode;

//Imports

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="DEREK - Full Controls  v11", group="THIS COMP")
@Disabled

public class TeleOpDrivingBetter extends LinearOpMode {

    //Hardware Initializing

    private  Servo                    clawClampServo;
    private  Servo                    capstoneServo;
    private  Servo                    zAxisClawSpinSevo;
    private  Servo                    leftFrontClamp;
    private  Servo                    rightFrontClamp;

    private  DcMotor                  frontLeftMotor;
    private  DcMotor                  frontRightMotor;
    private  DcMotor                  backLeftMotor;
    private  DcMotor                  backRightMotor;

    private  DcMotor                  yAxisClawMoveMotor;
    private  DcMotor                  pullySystemMotor;
    private  DcMotor                  leftIntakeWheel;
    private  DcMotor                  rightIntakeWheel;


    //Variable Initializing
    double speedMultiplier;


    @Override
    public void runOpMode() throws InterruptedException {
        //Code to run ONE TIME after the driver hits INIT

        //Variable Initialization

        //Hardware Mapping
        clawClampServo = hardwareMap.servo.get("clawClamp");
        zAxisClawSpinSevo = hardwareMap.servo.get("ClawSpin");
        capstoneServo = hardwareMap.servo.get("capstoneServo");
        rightFrontClamp = hardwareMap.servo.get("rightFrontClamp");
        leftFrontClamp = hardwareMap.servo.get("leftFrontClamp");

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        backRightMotor = hardwareMap.dcMotor.get("backRight");

        pullySystemMotor = hardwareMap.dcMotor.get("Vertical");
        yAxisClawMoveMotor = hardwareMap.dcMotor.get("Move");
        leftIntakeWheel = hardwareMap.dcMotor.get("LeftIntakeWheel");
        rightIntakeWheel = hardwareMap.dcMotor.get("RightIntakeWheel");


        //Hardware Initialization
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        pullySystemMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        yAxisClawMoveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntakeWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pullySystemMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        yAxisClawMoveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntakeWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntakeWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Set variables
        speedMultiplier = 1;


        //Wait for start
        telemetry.addData("Status: ", "Done");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP


                //GAMEPAD ONE
               // if (Math.abs(gamepad1.left_stick_x) >= 0.2 || Math.abs(gamepad1.left_stick_y) >= 0.2 || Math.abs(gamepad1.right_stick_x) >= 0.2) {
                //    mechinumJoystickDriving(); } else { slowModeDriving(); }
                foundationControls();
                //speedControl();
                intakeWheels();
                slowModeDriving();
                speedMultiplier = 1;


                //GAMEPAD TWO
                verticalSlide();
                horizontalSlide();
                clawMovement();
                capstoneDispenser();


                //TELEMTRY
                int frontLeftMotorDisplayPower   =  (int) Math.round(frontLeftMotor.getPower()  );
                int frontRightMotorDisplayPower  =  (int) Math.round(frontRightMotor.getPower() );
                int backLeftMotorDisplayPower    =  (int) Math.round(backLeftMotor.getPower()   );
                int backRightMotorDisplayPower   =  (int) Math.round(backRightMotor.getPower()  );

                telemetry.addLine( ( "FL: "  + frontLeftMotorDisplayPower   +  "  FR: " + frontRightMotorDisplayPower    ) );
                telemetry.addLine( ( "BL: "  + backLeftMotorDisplayPower    +  "  BR: " + backRightMotorDisplayPower     ) );
    //            telemetry.addLine( ( "LFS :" + leftFrontClamp.getPosition() + "  RFS :" + rightFrontClamp.getPosition()  ) );
                telemetry.addLine( ("Carson's Gay Magnet: "+"True" + "    Divya + Carson: " + "Viable"                  ) );
      //          telemetry.addLine( ("CS: " + capstoneServo.getPosition() + "ZAS: " + zAxisClawSpinSevo.getPosition()     ) );
                telemetry.update();

            }
        }
    }


    //Functions
    public void slowModeDriving() {
        if (this.gamepad1.dpad_up) {
            frontLeftMotor.setPower(-1);
            frontRightMotor.setPower(-1);
            backLeftMotor.setPower(-1);
            backRightMotor.setPower(-1);
        } else if (this.gamepad1.dpad_down) {
            frontLeftMotor.setPower(1);
            frontRightMotor.setPower(1);
            backLeftMotor.setPower(1);
            backRightMotor.setPower(1);
        } else if (this.gamepad1.dpad_left) {
            frontLeftMotor.setPower(-1);
            frontRightMotor.setPower(1);
            backLeftMotor.setPower(1);
            backRightMotor.setPower(-1);
        } else if (this.gamepad1.dpad_right) {
            frontLeftMotor.setPower(1);
            frontRightMotor.setPower(-1);
            backLeftMotor.setPower(-1);
            backRightMotor.setPower(1);
        } else if (this.gamepad1.left_bumper) {
            frontLeftMotor.setPower(1);
            frontRightMotor.setPower(-1);
            backLeftMotor.setPower(1);
            backRightMotor.setPower(-1);
        } else if (this.gamepad1.right_bumper) {
            frontLeftMotor.setPower(-1);
            frontRightMotor.setPower(1);
            backLeftMotor.setPower(-1);
            backRightMotor.setPower(1);
        } else {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }
    public void mechinumJoystickDriving() {
        double drive           =  this.gamepad1.left_stick_y;
        double strafe          =  this.gamepad1.left_stick_x;
        double rotate          =  this.gamepad1.right_stick_x;

        double roughFrontLeft  =  drive + strafe + rotate;
        double roughFrontRight =  drive - strafe - rotate;
        double roughBackLeft   =  drive - strafe + rotate;
        double roughBackRight  =  drive + strafe - rotate;

        double multiplier      =  1 / (Math.abs(drive) + Math.abs(strafe));

        double pureFrontLeft   =  roughFrontLeft  *  multiplier  *  speedMultiplier;
        double pureFrontRight  =  roughFrontRight *  multiplier  *  speedMultiplier;
        double pureBackLeft    =  roughBackLeft   *  multiplier  *  speedMultiplier;
        double pureBackRight   =  roughBackRight  *  multiplier  *  speedMultiplier;

        frontLeftMotor.setPower  ( pureFrontLeft  );
        frontRightMotor.setPower ( pureFrontRight );
        backLeftMotor.setPower   ( pureBackLeft   );
        backRightMotor.setPower  ( pureBackRight  );

    }
    public void speedControl() {
        if (this.gamepad1.y) {
            speedMultiplier = 1.00;
        }
        if (this.gamepad1.b) {
            speedMultiplier = 0.75;
        }
        if (this.gamepad1.a) {
            speedMultiplier = 0.50;
        }
    }
    public void intakeWheels() {
        float wheelPower = this.gamepad1.left_trigger - this.gamepad1.right_trigger;
        leftIntakeWheel.setPower(wheelPower);
        rightIntakeWheel.setPower(wheelPower);
    }
    public void foundationControls(){
        if (this.gamepad1.x){
            leftFrontClamp.setPosition(0);
            rightFrontClamp.setPosition(1);
        } else {
            leftFrontClamp.setPosition(1);
            rightFrontClamp.setPosition(0);
        }
    }
    public void verticalSlide(){
        pullySystemMotor.setPower(0.75 * this.gamepad2.right_stick_y);

    }
    public void horizontalSlide(){
        yAxisClawMoveMotor.setPower(0.75 * this.gamepad2.left_stick_y);
    }
    public void clawMovement(){
        if(this.gamepad2.left_bumper){
            clawClampServo.setPosition(0);
        }
        if(this.gamepad2.right_bumper){
            clawClampServo.setPosition(1);
        }

        zAxisClawSpinSevo.setPosition( zAxisClawSpinSevo.getPosition() + (0.1*this.gamepad2.left_stick_x) );

    }
    public void capstoneDispenser(){
        if (this.gamepad2.x || this.gamepad2.a || this.gamepad2.b || this.gamepad2.y){
            capstoneServo.setPosition(0);
        } else {
            capstoneServo.setPosition(1);
        }
    }

}