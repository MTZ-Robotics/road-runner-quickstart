package org.firstinspires.ftc.teamcode.MTZ;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MTZ.AutoControlsMTZ;

import java.io.IOException;

@Autonomous(name="basketside2", group ="A_Top")
//@Disabled
public class Auto2024BasketSide2 extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {
        try {
            super.autoPaths("Red","basket2",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
