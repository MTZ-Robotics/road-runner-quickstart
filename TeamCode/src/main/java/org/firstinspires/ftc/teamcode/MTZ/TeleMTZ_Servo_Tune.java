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
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.launcherReleasePosition;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.launcherSetPosition;
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

@TeleOp(name="TeleMTZ_Servo_Tune", group ="Test")
//@Disabled
/****
 * Use up and down arrows to tune servo values
 */

public class TeleMTZ_Servo_Tune extends LinearOpMode {
    double pos = 0.5;
    private Servo leftClaw;
    mtzButtonBehavior levelUpStatus = new mtzButtonBehavior();         //Move Hand to Next Level Higher
    mtzButtonBehavior levelDownStatus = new mtzButtonBehavior();         //Move Hand to Next Level Lower
    @Override
    public void runOpMode() throws InterruptedException {
        leftClaw = hardwareMap.servo.get("leftClaw");
        waitForStart();
        while (opModeIsActive()) {
            levelUpStatus.update(gamepad2.dpad_up);             //Move Hand to Next Level Higher
            levelDownStatus.update(gamepad2.dpad_down);             //Move Hand to Next Level Lower
            displayTelemetry();
            if (levelUpStatus.clickedDown) {
                pos = pos + 0.05;
            }
            if (levelDownStatus.clickedDown) {
                pos = pos - 0.05;
            }
            leftClaw.setPosition(pos);
        }
    }
//Telemetry Methods
    public void displayTelemetry() {
        telemetry.clearAll();

            telemetry.addLine()
                    .addData("leftClaw: ", leftClaw.getPosition());

        telemetry.update();
    }

    //End of Telemetry Methods
    //End of Class
}
