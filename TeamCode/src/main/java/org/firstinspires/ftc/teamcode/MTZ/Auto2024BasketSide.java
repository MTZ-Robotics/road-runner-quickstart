package org.firstinspires.ftc.teamcode.MTZ;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="basketside", group ="A_Top")
//@Disabled
public class Auto2024BasketSide extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {
        try {
            super.autoPaths("Red","basket",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
