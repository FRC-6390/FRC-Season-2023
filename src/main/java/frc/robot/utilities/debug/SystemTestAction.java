package frc.robot.utilities.debug;

import java.util.function.DoubleConsumer;

public record SystemTestAction(DoubleConsumer action, double precent){
}