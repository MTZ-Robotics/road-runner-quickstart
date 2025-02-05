package org.firstinspires.ftc.teamcode.MTZ;

import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.armExtensionCollapsedLength;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.armExtensionInchesAtHome;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.armLengthDesired;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.armPivotHeight;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.armRotationDegreesAtHome;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.defaultArmAssistLevel;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.defaultArmExtensionPower;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.defaultArmLowerPower;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.defaultArmPower;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.defaultDriveSpeed;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.defaultPauseTime;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.driveBump;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.driveFastRatio;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.driveSlowRatio;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.endGameOver;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.endGameStart;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.endGameWarning;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.endGameWarning2;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.findStackDistance;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.findStackLevel;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.greenWarningTime;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.handAssistRideHeightAboveLevel;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.handAssistRideHeightDistance;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.handAssistRideHeightLevel;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.handRotatorNormalPosition;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.leftClawClosedPosition;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.leftClawOpenPosition;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.maxArmDegrees;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.maxArmExtensionInches;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.maxWristPosition;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.minArmDegrees;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.minArmExtensionInches;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.minWristPosition;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.prorate;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.redWarningTime;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.rightClawClosedPosition;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.rightClawOpenPosition;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.scoopStage;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.stackDistanceArray;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.stackDistanceAtHome;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.stackHeightAboveLevelArray;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.stackHeightOnLevelArray;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.stackLevelAtHome;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.strafeBump;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.ticksPerDegreeArm;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.ticksPerDegreeTurnChassis;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.ticksPerInchExtension;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.ticksPerInchWheelDrive;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.ticksPerInchWheelStrafe;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.turnBump;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.wristAdjustment;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.wristBump;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.wristConversionToServo;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.yellowWarningTime;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleMTZ_Drive_ItD_Elevator", group ="Core")

//@Disabled

/****
 * This class is intended to be a sub class to run the robot with the controllers.
 * The super classes may call out different controller maps to use
 *
 *
 *
 *
 */

public class TeleMTZ_Drive_Controls_ItD_Elevator extends LinearOpMode {

    /********************************
     * Robot Configuration Flags
     ********************************/
    boolean accountForArmDrift;
    boolean hasChassisMotors;
    boolean hasAuxMotorsAndServos;
    boolean hasLightsHub;
    boolean wantAutoChassisControls;
    boolean leftClawRemainClosed = true;
    boolean rightClawRemainClosed = true;
    private int allianceReverser = 1;
    double armAssistLevel = defaultArmAssistLevel;
    double handRotatorPosition = handRotatorNormalPosition;

    /********************************
     * Timer Variables
     ********************************/
    private ElapsedTime endGameTimer;

    boolean greenTimerElapsed;
    boolean yellowTimerElapsed;
    boolean redTimerElapsed;
    boolean endGameStartElapsed;
    double tempLightsTimer;

    /***********
     * Lights Control Declarations
     ***********/

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    RevBlinkinLedDriver.BlinkinPattern tempLightsPattern;

    /*************************
     * Motor & Servo Variables
     *************************/
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor arm;
    private DcMotor elevator;
    private Servo leftClaw;
    private Servo rightClaw;
    private Servo wrist;
    private Servo handRotator;
    double drivePower;
    double blPower;
    double brPower;
    double flPower;
    double frPower;
    double powerRatio=1;

    boolean aboveLevel = false;
    boolean stackingDown;

    int tagAdditional = 0;

    int stackLevel = stackLevelAtHome;
    int stackDistance = stackDistanceAtHome;
    double armRotationDegrees = armRotationDegreesAtHome;
    double armExtensionInches = armExtensionInchesAtHome;
    double verticalDesired;
    double horizontalDesired;
    double stackDegreesDesired;
    String debugString ="none";


    double wristPositionDesired = wristConversionToServo(armRotationDegreesAtHome) + wristAdjustment;


    /*******
     * Add Controller Variables & Objects
     ********/


    /****Blue*********           IntoTheDeep Elevator     Control Pad Map            *******Blue*******/
// Assign Variables & Objects for Control Pads
    double chassisSpeedSlow;                             //Slow Speed
    mtzButtonBehavior chassisBumpLeftTurnStatus = new mtzButtonBehavior();         //Bump Left Turn




    mtzButtonBehavior chassisBumpForwardStatus = new mtzButtonBehavior();         //Bump Forward
    mtzButtonBehavior chassisBumpLeftStrafeStatus = new mtzButtonBehavior();         //Bump Left Strafe
    mtzButtonBehavior chassisBumpRightStrafeStatus = new mtzButtonBehavior();         //Bump Right Strafe
    mtzButtonBehavior chassisBumpBackStatus = new mtzButtonBehavior();         //Bump Backwards


    double driveStick2;                             //Drive 2
    double turnStick;                             //Turn
    double chassisSpeedFast;                             //High Speed
    mtzButtonBehavior chassisBumpRightTurnStatus = new mtzButtonBehavior();         //Bump Right Turn

    mtzButtonBehavior startButton1Status = new mtzButtonBehavior();         //Pad Select (A & B)








    double driveStick1;                             //Drive 1
    double strafeStick;                             //Strafe
    double armClawClose;                             //Arm Claw Close (Sticky)
    mtzButtonBehavior armClawOpenStatus = new mtzButtonBehavior();         //Arm Claw Open (Sticky)




    mtzButtonBehavior wristUpStatus = new mtzButtonBehavior();         //Wrist Up
    mtzButtonBehavior clawRotateLeftStatus = new mtzButtonBehavior();         //Claw Rotate Left
    mtzButtonBehavior clawRotateRightStatus = new mtzButtonBehavior();         //Claw Rotate Right
    mtzButtonBehavior wristDownStatus = new mtzButtonBehavior();         //Wrist Down


    double armExtendStick;                             //Arm Extend

    double elevatorClawClose;                             //Elevator Claw Close (Sticky)
    mtzButtonBehavior elevatorClawOpenStatus = new mtzButtonBehavior();         //Elevator Claw Open (Sticky)

    mtzButtonBehavior startButton2Status = new mtzButtonBehavior();         //Pad Select (A & B)



    mtzButtonBehavior rotateElevatorLeftStatus = new mtzButtonBehavior();         //Rotate Elevator Left
    mtzButtonBehavior rotateElevatorRightStatus = new mtzButtonBehavior();         //Rotate Elevator Right



    double elevatorStick;                             //Elevator Stick

// End of Assignment Mapping
    /****Blue*********           End     IntoTheDeep Elevator     Control Pad Map            ******Blue********/



    @Override

    //This is the default opMode call for generically running the opMode in this class directly from the phone without calling it from a super class
    public void runOpMode() throws InterruptedException{
        /******************************************************
         * Default
         *
         * These default settings will be used if THIS opMode is selected from the driver station.
         * Typically this opMode is not called since this class is used by super classes to call the controlRobot method with specific variables.
         * It is helpful to use this opMode for testing the controlRobot method
         *****************************************************/

        controlRobot("Red", "Center Stage R1", defaultDriveSpeed, true, true, true, true);
    }

    //This is the method that handles the controls
    public void controlRobot(String alliance, String controlPadMap, Double defaultDrivePower, Boolean runChassis, Boolean runAux, Boolean runLights, Boolean runAutos) throws InterruptedException {

        // Robot Configuration Flags
        hasChassisMotors = runChassis;
        hasAuxMotorsAndServos = runAux;
        hasLightsHub = runLights;
        wantAutoChassisControls = runAutos;
        // non-counterbalanced Spur Gear arm needs to account for arm drift
        accountForArmDrift = false;

        //Code written for Blue alliance and reverse turns if on Red alliance
        allianceReverser=1;
        if (alliance=="Red") {
            allianceReverser=-1;
        }

        /***********************
         * Modifiable variables
         **********************/

        /***************
         * reset Timer Variables to false
         ***************/
        greenTimerElapsed = false;
        yellowTimerElapsed = false;
        redTimerElapsed = false;
        endGameStartElapsed = false;

        /*************
         * Assign Lights Variables
         *************/
       if(hasLightsHub) {
           blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

           if (alliance=="Red") {
               pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
           } else {
               pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE;
           }
           blinkinLedDriver.setPattern(pattern);
       }

        /*******************************
         * Assign Motor & Servo Variables
         ******************************/
        if(hasChassisMotors){
            frontLeft = hardwareMap.dcMotor.get("frontLeft");
            frontRight = hardwareMap.dcMotor.get("frontRight");
            backLeft = hardwareMap.dcMotor.get("backLeft");
            backRight = hardwareMap.dcMotor.get("backRight");
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(hasAuxMotorsAndServos){
            leftClaw = hardwareMap.servo.get("leftClaw");
            //leftClaw.setDirection(Servo.Direction.REVERSE);
            rightClaw = hardwareMap.servo.get("rightClaw");
            wrist = hardwareMap.servo.get("wrist");
            handRotator = hardwareMap.servo.get("handRotator");
            arm = hardwareMap.dcMotor.get("arm");
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //arm.setDirection(DcMotor.Direction.REVERSE);
            elevator = hardwareMap.dcMotor.get("elevator");
            elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevator.setDirection(DcMotor.Direction.REVERSE);
            elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            handRotator.setDirection(Servo.Direction.REVERSE);
        }


        /**********************************
         * Do Not set positions on initialize since that counts as controlling the robot
         * and initialize would not be able to happen until the timer starts for driver controlled period
         **********************************/


        /***********************************************
         * Tell driver station that initialization is complete
         **********************************************/
        telemetry.log().clear();
        telemetry.update();
        telemetry.log().add("Initialized. Control Pad Map = "+controlPadMap+". Go "+alliance+" alliance");

        telemetry.log().add(greenWarningTime+", " +
                yellowWarningTime+", " +
                redWarningTime+", " +
                endGameStart
        );

        wristAdjustment = 0.0;
        /************* Press Play Button ***********************/

        waitForStart();

        /************ START ***************/

        if(hasLightsHub) {
            //Use a gentle pattern during the normal part of the match
            //pattern = RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE;
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            blinkinLedDriver.setPattern(pattern);
        }
        //Start timer here since play was just pressed
        endGameTimer = new ElapsedTime();
        endGameTimer.reset();

        while (opModeIsActive()) {
            /**************************************************************
             *
             * TeleOp Loops From Here to the End of controlRobot
             *
             * Loops often to see if controls are still the same
             *
             ****************************************************************/

            /***********************
             * Gather Button Input *
             **********************/


/****Green*********           IntoTheDeep Elevator     Controls Update Status           *******Green*******/

            if (controlPadMap=="IntoTheDeep Elevator") {

                chassisSpeedSlow = gamepad1.left_trigger;             //Slow Speed
                chassisBumpLeftTurnStatus.update(gamepad1.left_bumper);             //Bump Left Turn




                chassisBumpForwardStatus.update(gamepad1.dpad_up);             //Bump Forward
                chassisBumpLeftStrafeStatus.update(gamepad1.dpad_left);             //Bump Left Strafe
                chassisBumpRightStrafeStatus.update(gamepad1.dpad_right);             //Bump Right Strafe
                chassisBumpBackStatus.update(gamepad1.dpad_down);             //Bump Backwards


                driveStick2 = gamepad1.left_stick_y;             //Drive 2
                turnStick = gamepad1.left_stick_x;             //Turn
                chassisSpeedFast = gamepad1.right_trigger;             //High Speed
                chassisBumpRightTurnStatus.update(gamepad1.right_bumper);             //Bump Right Turn

                startButton1Status.update(gamepad1.start);             //Pad Select (A & B)








                driveStick1 = gamepad1.right_stick_y;             //Drive 1
                strafeStick = gamepad1.right_stick_x;             //Strafe
                armClawClose = gamepad2.left_trigger;             //Arm Claw Close (Sticky)
                armClawOpenStatus.update(gamepad2.left_bumper);             //Arm Claw Open (Sticky)




                wristUpStatus.update(gamepad2.dpad_up);             //Wrist Up
                clawRotateLeftStatus.update(gamepad2.dpad_left);             //Claw Rotate Left
                clawRotateRightStatus.update(gamepad2.dpad_right);             //Claw Rotate Right
                wristDownStatus.update(gamepad2.dpad_down);             //Wrist Down


                armExtendStick = gamepad2.left_stick_y;             //Arm Extend

                elevatorClawClose = gamepad2.right_trigger;             //Elevator Claw Close (Sticky)
                elevatorClawOpenStatus.update(gamepad2.right_bumper);             //Elevator Claw Open (Sticky)

                startButton2Status.update(gamepad2.start);             //Pad Select (A & B)



                rotateElevatorLeftStatus.update(gamepad2.x);             //Rotate Elevator Left
                rotateElevatorRightStatus.update(gamepad2.b);             //Rotate Elevator Right



                elevatorStick = gamepad2.right_stick_y;             //Elevator Stick

            }     /*****Green********           End     IntoTheDeep Elevator     Updates            ******Green********/



            else {

                /***********************************
                 * Control Pad Map Selection Error *
                 **********************************/
                telemetry.log().add("Error in Control Map Selection"); telemetry.update();
                if(hasLightsHub){pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_RED; blinkinLedDriver.setPattern(pattern);}
                waitForStart(); sleep(30000);
            }

            displayTelemetry();

            /*
            if(startButton2Status.isDown && resetHomeStatus.isDown){
                stackLevelAtHome = stackLevel;
                stackDistanceAtHome = stackDistance;
                armRotationDegreesAtHome = armRotationDegrees;
                armExtensionInchesAtHome =  armExtensionInches;
                wristAdjustment=0;
            }

             */


            /**********************************************************************************************
             * Speed adjust with triggers                                                                 *
             * If the chassisSpeedFast trigger is pulled, the motor speed will increase a constant amount *
             * If the chassisSpeedSlow trigger is pulled, the motor speed will decrease a constant amount *
             * chassisSpeedFast overrides chassisSpeedSlow                                                *
             *********************************************************************************************/
            if (chassisSpeedFast > 0) {
                powerRatio = driveFastRatio;
            } else if (chassisSpeedSlow > 0) {
                powerRatio = driveSlowRatio;
            } else {

                powerRatio = 1.0;
            }
            drivePower = defaultDrivePower*powerRatio;

            /**************************
             * Chassis drive controls *
             *************************/

            turnStick = turnStick * .85; //Turns are too fast, was 1.0
            blPower = drivePower * ((-driveStick2 + -driveStick1 + strafeStick) - turnStick);
            brPower = drivePower * ((-driveStick2 + -driveStick1 - strafeStick) + turnStick);
            flPower = drivePower * ((-driveStick2 + -driveStick1 + strafeStick) + turnStick);
            frPower = drivePower * ((-driveStick2 + -driveStick1 - strafeStick) - turnStick);
            if(!runAux) {
                blPower = drivePower * ((-driveStick2 + -driveStick1 + strafeStick) + turnStick);
                brPower = drivePower * ((-driveStick2 + -driveStick1 - strafeStick) - turnStick);
                flPower = drivePower * ((-driveStick2 + -driveStick1 + strafeStick) - turnStick);
                frPower = drivePower * ((-driveStick2 + -driveStick1 - strafeStick) + turnStick);
            }

            else {
                //Set motors to run manually from controller
                backLeft.setPower(blPower);
                backRight.setPower(brPower);
                frontLeft.setPower(flPower);
                frontRight.setPower(frPower);
            }            /*************************
             * Chassis bump controls
             *************************/
            if(hasChassisMotors && wantAutoChassisControls) {
                if (chassisBumpForwardStatus.clickedDown) {
                    Drive(driveBump, .5, 0);
                }
                if (chassisBumpBackStatus.clickedDown) {
                    Drive(-driveBump, .5, 0);
                }
                if (chassisBumpLeftStrafeStatus.clickedDown) {
                    Strafe(strafeBump, .5, 0);
                }
                if (chassisBumpRightStrafeStatus.clickedDown) {
                    Strafe(-strafeBump, .5, 0);
                }
                if (chassisBumpLeftTurnStatus.clickedDown) {
                    Turn(-turnBump, .5, 0);
                }
                if (chassisBumpRightTurnStatus.clickedDown) {
                    Turn(turnBump, .5, 0);
                }
            }


            /*************************
             * Aux drive controls
             *************************/
            /*************
             * Arm Controls
             *************/
            //Arm Out & In
            if(hasAuxMotorsAndServos) {
                arm.setPower(defaultArmPower * powerRatio * (-armExtendStick));

                //Elevator Up and Down

                elevator.setPower((-elevatorStick) * powerRatio);


                //handAssist
/*
                if (returnHomeStatus.clickedDown) {
                    // Zero Height Desired
                    stackLevel = 0;
                    stackDistance = 0;
                    aboveLevel = false;
                    goToStackPosition(false, stackLevel, stackDistance, aboveLevel);
                }

 */


                /************************
                 * Stacker Controls
                 ***********************/
                armRotationDegrees = (arm.getCurrentPosition() / mtzConstants_ItD.ticksPerDegreeArm) + armRotationDegreesAtHome;
                armExtensionInches = elevator.getCurrentPosition() / ticksPerInchExtension - armExtensionInchesAtHome;

/*
                if (levelUpStatus.clickedDown) {
                    stackingDown = false;
                    if (stackLevel != -1 && stackLevel < stackHeightOnLevelArray.length - 1) {
                        stackLevel++;
                        stackDistance = stackLevel;
                    }
                    goToStackPosition(stackingDown, stackLevel, stackDistance, aboveLevel);
                }
                if (levelDownStatus.clickedDown) {
                    stackingDown = true;
                    if (stackLevel != -1) {
                        if (aboveLevel) {
                            //aboveLevel = false;
                        } else if (stackLevel != 0) {
                            stackLevel--;
                            stackDistance = stackLevel;
                            //aboveLevel = true;
                        }
                    }
                    goToStackPosition(stackingDown, stackLevel, stackDistance, aboveLevel);
                }

 */

                /*************
                 * Wrist Adjuster
                 *************/
                /*
                if (wristAdjustLessStatus.clickedDown) {
                    wristAdjustment = wristAdjustment - wristBump;
                } else if (wristAdjustMoreStatus.clickedDown) {
                    wristAdjustment = wristAdjustment + wristBump;
                }

                 */

                /********************************************
                 *
                 * Wrist Angle Auto Set
                 *
                 * without stacker controls
                 *
                 ********************************************/
                if (armRotationDegrees > 15 && armRotationDegrees < 50) {//Deliver on the backdrop at the front of the robot
                    wristPositionDesired = wristAutoLevelDeliverFront(armRotationDegrees) + wristAdjustment;
                    scoopStage = 2;
                    debugString = "Front " + wristPositionDesired + "";

                } else if (armRotationDegrees > 50) {//Deliver over the top to the rear
                    wristPositionDesired = wristAutoLevelDeliverRear(armRotationDegrees) + wristAdjustment;
                    scoopStage = 3;
                    debugString = "Rear " + wristPositionDesired + "";

                } else {//Scoop
                    wristPositionDesired = wristAutoLevelScoop(armRotationDegrees) + wristAdjustment;
                    scoopStage = 1;
                    debugString = "Scoop " + wristPositionDesired + "";

                }

                if (wristPositionDesired < minWristPosition) {
                    wristPositionDesired = minWristPosition;
                }

                if (wristPositionDesired > maxWristPosition) {
                    wristPositionDesired = maxWristPosition;
                }

                //Set wrist position
                wrist.setPosition(wristPositionDesired);

                /*************
                 * Claw Controls
                 *************/

                if (armClawClose > 0.95) {
                    leftClawRemainClosed = true;
                }
                if (armClawOpenStatus.clickedDown) {
                    leftClawRemainClosed = false;
                }
                if (elevatorClawClose > 0.95) {
                    rightClawRemainClosed = true;
                }
                if (elevatorClawOpenStatus.clickedDown) {
                    rightClawRemainClosed = false;
                }
                if (leftClawRemainClosed) {
                    leftClaw.setPosition(leftClawClosedPosition);
                } else {
                    //Close claw to prorated level of between open and closed position based on the current trigger value between open trigger and closed trigger
                    leftClaw.setPosition(prorate(armClawClose, 0, 1, leftClawOpenPosition, leftClawClosedPosition));
                }
                if (rightClawRemainClosed) {
                    rightClaw.setPosition(rightClawClosedPosition);
                } else {
                    //Close claw to prorated level of between open and closed position based on the current trigger value between open trigger and closed trigger
                    rightClaw.setPosition(prorate(elevatorClawClose, 0, 1, rightClawOpenPosition, rightClawClosedPosition));
                }
            }


            /*********************************
             * Check if timer has elapsed
             *********************************/


            if(hasLightsHub) {
                //Check for End Timer First
                if (endGameTimer.seconds() < tempLightsTimer){
                    blinkinLedDriver.setPattern(tempLightsPattern);
                } else if (endGameTimer.seconds() > endGameOver) {
                    endGameStartElapsed = true;
                    pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
                    blinkinLedDriver.setPattern(pattern);
                } else if (endGameTimer.seconds() > endGameWarning2) {
                    endGameStartElapsed = true;
                    pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
                    blinkinLedDriver.setPattern(pattern);
                } else if (endGameTimer.seconds() > endGameWarning) {
                    endGameStartElapsed = true;
                    pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    blinkinLedDriver.setPattern(pattern);
                } else if (endGameTimer.seconds() > endGameStart) {
                    endGameStartElapsed = true;
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                    blinkinLedDriver.setPattern(pattern);
                } else if (endGameTimer.seconds() > redWarningTime) { //Then check for red
                    redTimerElapsed = true;
                    pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    blinkinLedDriver.setPattern(pattern);
                } else if (endGameTimer.seconds() > yellowWarningTime) { //Then check for yellow
                    yellowTimerElapsed = true;
                    pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                    blinkinLedDriver.setPattern(pattern);
                } else if (endGameTimer.seconds() > greenWarningTime) { //Then check for green
                    greenTimerElapsed = true;
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    blinkinLedDriver.setPattern(pattern);
                }

                //Stop supporting arm 5 seconds after end game
                if (endGameTimer.seconds() > endGameOver + 5) {
                    armAssistLevel = 0;
                }
            }
        }
    }

    /*******************************
     * End of Control Robot Method
     ******************************/

    /****************************************************************************
     * Motion Methods                                                           *
     * These methods cause motion in the robot when called by the program above *
     ***************************************************************************/

    /*****************
     * Drive
     *
     * Propels the robot forward or backward a certain number of inches with all 4 wheels turning the same direction
     *
     * @param distance inches
     * @param motorPower
     * @param pause
     * @throws InterruptedException
     */
    public void Drive(double distance, double motorPower, int pause) throws InterruptedException {
        if(hasChassisMotors) {
            if (opModeIsActive()) {
                StopAndResetDriveEncoders();
                DriveByInches(distance);
                RunDriveToPosition();
                DrivePower(motorPower);
                while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    DisplayDriveTelemetry();
                }
                Thread.sleep(pause);
                RunDriveWithEncoders();
            }
        }
    }

    /***********
     * Strafe
     *
     * Propels the robot sideways a certain number of inches
     *
     * @param leftDistance inches
     * @param power
     * @param pause
     * @throws InterruptedException
     */
    public void Strafe(double leftDistance, double power, int pause) throws InterruptedException {
        if(hasChassisMotors) {
            //Left is positive
            if (opModeIsActive()) {
                StopAndResetDriveEncoders();
                StrafeByInches(leftDistance);
                RunDriveToPosition();
                DrivePower(power);
                while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    DisplayDriveTelemetry();
                }
                Thread.sleep(pause);

                RunDriveWithEncoders();
            }
        }
    }

    /*******
     * Turn
     *
     * Rotates the robot about the center of the 4 wheels
     * Left is negative degrees
     *
     * @param rightDegrees
     * @param power
     * @param pause
     * @throws InterruptedException
     */
    public void Turn(double rightDegrees, double power, int pause) throws InterruptedException {
        if(hasChassisMotors) {
            //Left is negative
            if (opModeIsActive()) {
                StopAndResetDriveEncoders();
                TurnByAngle(rightDegrees);
                RunDriveToPosition();
                DrivePower(power);
                while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    DisplayDriveTelemetry();
                }
                Thread.sleep(pause);

                RunDriveWithEncoders();
            }
        }
    }

    //Arm Methods

    public void goToStackPosition(boolean stackingDown,int stackLevel,int stackDistance,boolean aboveLevel) throws InterruptedException {
        double vertDesired, horDesired, degreesDesired, vertRequired,armSpeed;
        if (stackLevel<0){
            stackLevel = findStackLevel();
            if(stackingDown){
                aboveLevel = true;
            }
        }
        if (stackDistance<0){
            stackDistance = findStackDistance();
        }
        if(stackLevel>stackHeightOnLevelArray.length-1){
            stackLevel = stackHeightOnLevelArray.length-1;
        }
        if(stackDistance>stackDistanceArray.length-1){
            stackDistance = stackDistanceArray.length-1;
        }
        if (stackLevel >= 0 && stackDistance >= 0) {
            // Check if the stone is getting set down on a level and go slow if so
            if (aboveLevel) {
                vertDesired = stackHeightAboveLevelArray[stackLevel];
                armSpeed = mtzConstants_ItD.defaultArmLowerPower;
            } else {
                vertDesired = stackHeightOnLevelArray[stackLevel];
                armSpeed = mtzConstants_ItD.defaultArmLowerPower / 3;
            }
            if (!stackingDown) {
                armSpeed = mtzConstants_ItD.defaultArmPower;
            }

            vertRequired = vertDesired - armPivotHeight;
            horDesired = stackDistanceArray[stackDistance];

            degreesDesired = Math.toDegrees(Math.asin((vertRequired) / (armLengthDesired(horDesired, vertDesired))));

            //Stay in the max & min
            armExtensionInches = armLengthDesired(horDesired, vertDesired) - armExtensionCollapsedLength;
            if(armExtensionInches < minArmExtensionInches){
                armExtensionInches = minArmExtensionInches;
            } else if(armExtensionInches > maxArmExtensionInches){
                armExtensionInches = maxArmExtensionInches;
            }
            if(degreesDesired < minArmDegrees){
                degreesDesired = minArmDegrees;
            } else if(degreesDesired > maxArmDegrees){
                degreesDesired = maxArmDegrees;
            }
            stackDegreesDesired = degreesDesired;
            // Set the target positions to run to
            raiseByDegrees(degreesDesired);
            //wristPositionDesired = wristAutoLevel(degreesDesired);
            horizontalDesired = horDesired;
            verticalDesired = vertDesired;
            if(hasAuxMotorsAndServos) {
                elevator.setTargetPosition((int) ((armExtensionInches - armExtensionInchesAtHome) * ticksPerInchExtension));

                if (opModeIsActive()) {
                    // Turn motors on to let them reach the target if the stop button hasn't been pressed
                    arm.setPower(armSpeed);
                    elevator.setPower(defaultArmExtensionPower);
                    //Wrist Position is set once so it doesn't try to go to 2 different positions each loop iteration
                    // wrist.setPosition(wristPositionDesired);
                    while (arm.isBusy() || elevator.isBusy()) {
                        DisplayArmTelemetry();
                    }
                }

            }
            Thread.sleep(defaultPauseTime);
        }
    }

    public void RaiseArm(double degrees, double power,int pause) throws InterruptedException {
        if(hasAuxMotorsAndServos) {
            if (opModeIsActive()) {
                raiseByDegrees(degrees);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmPower(power);
                while (arm.isBusy() || elevator.isBusy()) {
                    DisplayArmTelemetry();
                }
            }
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Thread.sleep(pause);
        }
    }
    public void LowerArm(double degrees, double power, int pause) throws InterruptedException {
        if(hasAuxMotorsAndServos) {
            if (opModeIsActive()) {
                raiseByDegrees(-degrees);
                ArmPower(power);
                while (arm.isBusy() || elevator.isBusy()) {
                    DisplayArmTelemetry();
                }
            }
            Thread.sleep(pause);
        }
    }
    public double wristAutoLevelScoop(double armAngle){
        return wristConversionToServo(55 - armAngle);
    }
    public double wristAutoLevelDeliverFront(double armAngle){
        return wristConversionToServo(140 - armAngle);
    }
    public double wristAutoLevelDeliverRear(double armAngle){
        return wristConversionToServo(armAngle + 95); //30 was the initial guess, added 65 when mechanical interference was causing an issue
    }

    public void ExtendArm(double desiredArmLength, double power,int pause) throws InterruptedException {
        if (opModeIsActive()) {
            armExtensionInches = desiredArmLength - armExtensionCollapsedLength;
            if(armExtensionInches<minArmExtensionInches){
                armExtensionInches=minArmExtensionInches;
            } else if(armExtensionInches>maxArmExtensionInches){
                armExtensionInches=maxArmExtensionInches;
            }
            if(hasAuxMotorsAndServos) {
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition((int) (armExtensionInches * ticksPerInchExtension));
                elevator.setPower(power);

                while (arm.isBusy() || elevator.isBusy()) {
                    DisplayArmTelemetry();
                }
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        Thread.sleep(pause);
    }

//Encoder Methods

    public void StopAndResetAllEncoders() {
        if(hasChassisMotors) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if(hasAuxMotorsAndServos){
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void StopAndResetDriveEncoders() {
        if(hasChassisMotors) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void RunDriveToPosition() {
        if(hasChassisMotors) {
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void RunDriveWithEncoders() {
        if(hasChassisMotors) {
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void RunArmToPosition() {
        if(hasAuxMotorsAndServos) {
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void RunArm() {
        if(hasAuxMotorsAndServos) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    //End of Encoder Methods

//Distance Calculation Methods

    public void DriveByInches(double distance) {
        if(hasChassisMotors) {
            frontLeft.setTargetPosition((int) (distance * ticksPerInchWheelDrive));
            frontRight.setTargetPosition((int) (distance * ticksPerInchWheelDrive));
            backLeft.setTargetPosition((int) (-1 * distance * ticksPerInchWheelDrive));
            backRight.setTargetPosition((int) (-1 * distance * ticksPerInchWheelDrive));
        }
    }

    public void StrafeByInches(double distance) {
        if(hasChassisMotors) {
            frontLeft.setTargetPosition((int) (distance * ticksPerInchWheelStrafe));
            frontRight.setTargetPosition((int) (-distance * ticksPerInchWheelStrafe));
            backLeft.setTargetPosition((int) (distance * ticksPerInchWheelStrafe));
            backRight.setTargetPosition((int) (-distance * ticksPerInchWheelStrafe));
        }
    }

    public void TurnByAngle(double degrees) {
        if(hasChassisMotors) {
            frontLeft.setTargetPosition((int) (degrees * ticksPerDegreeTurnChassis));
            frontRight.setTargetPosition((int) (-degrees * ticksPerDegreeTurnChassis));
            backLeft.setTargetPosition((int) (-degrees * ticksPerDegreeTurnChassis));
            backRight.setTargetPosition((int) (degrees * ticksPerDegreeTurnChassis));
        }
    }
    public void raiseByDegrees(double degrees) {
        if(hasAuxMotorsAndServos){
            arm.setTargetPosition((int)((degrees + armRotationDegreesAtHome) * mtzConstants_ItD.ticksPerDegreeArm));
        }
    }

    //End of distance calculation methods

//Power Methods

    public void DrivePower(double power) {
        if(hasChassisMotors) {
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        }
    }
    public void ArmPower(double power) {
        if(hasAuxMotorsAndServos) {
            arm.setPower(power);
        }
    }
//End Power Methods


//Telemetry Methods
    public void displayTelemetry() {
        telemetry.clearAll();
        telemetry.addLine()
                .addData("Timer: ", endGameTimer.toString());
        telemetry.addLine()
                .addData("Blinkin: ", pattern);
        telemetry.addLine()
                .addData("Default Arm: ", defaultArmPower);
        if(hasAuxMotorsAndServos) {
            telemetry.addLine()
                    .addData("Arm Power: ", arm.getPower());
            telemetry.addLine()
                    .addData("Arm Extension Power: ", elevator.getPower());
            telemetry.addLine()
                    .addData("Arm Rotation: ", armRotationDegrees);
            telemetry.addLine()
                    .addData("leftClaw: ", leftClaw.getPosition());
            telemetry.addLine()
                    .addData("rightClaw: ", rightClaw.getPosition());
            telemetry.addLine()
                    .addData("Scoop Stage: ", scoopStage);
            telemetry.addLine()
                    .addData("wrist: ", wrist.getPosition());
            telemetry.addLine()
                    .addData("Plane Launch: ", handRotator.getPosition());
        }
        telemetry.addLine()
                .addData("Debug String: ", debugString);


        if(hasChassisMotors) {
            telemetry.addLine()
                    .addData("Front Left Power: ", frontLeft.getPower());
            telemetry.addLine()
                    .addData("Front Right Power: ", frontRight.getPower());
            telemetry.addLine()
                    .addData("Back Left Power: ", backLeft.getPower());
            telemetry.addLine()
                    .addData("Back Right Power: ", backRight.getPower());
        }
        telemetry.update();
    }
    public void DisplayDriveTelemetry() {
        if(hasChassisMotors) {
            double frontLeftInches = frontLeft.getCurrentPosition() / ticksPerInchWheelDrive;
            double frontRightInches = frontRight.getCurrentPosition() / ticksPerInchWheelDrive;
            double backLeftInches = backLeft.getCurrentPosition() / ticksPerInchWheelDrive;
            double backRightInches = backRight.getCurrentPosition() / ticksPerInchWheelDrive;
            telemetry.clear();
            telemetry.addLine()
                    .addData("F Left : ", (int) frontLeftInches + "in, Power: " + "%.1f", frontLeft.getPower());
            telemetry.addLine()
                    .addData("F Right: ", (int) frontRightInches + "in, Power: " + "%.1f", frontRight.getPower());
            telemetry.addLine()
                    .addData("B Left : ", (int) backLeftInches + "in, Power: " + "%.1f", backLeft.getPower());
            telemetry.addLine()
                    .addData("B Right: ", (int) backRightInches + "in, Power: " + "%.1f", backRight.getPower());
            if(hasAuxMotorsAndServos) {
                telemetry.addLine()
                        .addData("Arm: ", (int) arm.getCurrentPosition() / ticksPerDegreeArm + "deg, Power: " + "%.1f", arm.getPower());
            }
            telemetry.update();
        }

    }
    public void DisplayArmTelemetry() {
        if(hasAuxMotorsAndServos) {
            double armDegrees = arm.getCurrentPosition() / mtzConstants_ItD.ticksPerDegreeArm;
            telemetry.clear();
            telemetry.addLine()
                    .addData("Arm Degrees ", (int) armDegrees + "  Power: " + "%.1f", arm.getPower());
            telemetry.addLine()
                    .addData("Arm Ext Inches ", (int) armExtensionInches + "  Power: " + "%.1f", elevator.getPower());
            telemetry.update();
        }
    }
    //End of Telemetry Methods
    //End of Class
}
