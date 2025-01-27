package org.firstinspires.ftc.teamcode.MTZ;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.MTZ.AutoControlsMTZ;

import java.io.IOException;

@Autonomous(name="auto2024", group ="A_Top")
@Disabled
public class Auto2024 extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {
        try {
            super.autoPaths("Red","auto2024",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
