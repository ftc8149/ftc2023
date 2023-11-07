package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.lang.annotation.Annotation;

@Disabled
public class AutoTest implements Autonomous {
    @Override
    public String name() {
        return "Auto 1";
    }

    @Override
    public String group() {
        return null;
    }

    @Override
    public String preselectTeleOp() {
        return "TeleOp 1";
    }

    @Override
    public Class<? extends Annotation> annotationType() {
        return null;
    }
}
