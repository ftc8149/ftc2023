package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.annotation.Annotation;

@Disabled
public class TeleOpTest implements TeleOp {
    @Override
    public String name() {
        return "TeleOp 1";
    }

    @Override
    public String group() {
        return null;
    }

    @Override
    public Class<? extends Annotation> annotationType() {
        return null;
    }
}
