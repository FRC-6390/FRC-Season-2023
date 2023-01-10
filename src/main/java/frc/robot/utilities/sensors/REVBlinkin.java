package frc.robot.utilities.sensors;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class REVBlinkin {
    
    private Spark blinkin;
    private REVColour color;
    

    public REVBlinkin(int port){
        blinkin = new Spark(port);
    }

    public void setColour(int id){
        setColour(REVColour.getByID(id));
    }

    public void setColour(REVColour color){
        this.color = color;
        blinkin.set(color.getValue());
    }

    public REVColour getColour(){
        return color;
    }

}


enum REVColour{

    fallback(0,0.93), //defualt / fallback colour white
    Rainbow(1,-0.99),
    Party(2,-0.97),
    Ocean(3,-0.95),
    Lave(4,-0.93),
    Forest(5,-0.91),
    Rainbow_Glitter(6,-0.89),
    Confetti(7,-0.87),
    Shot_Red(8,-0.85),
    Shot_Blue(9,-0.83),
    Shot_White(10,-0.81),
    Sinelon_Rainbow(11,-0.79),
    Sinelon_Party(12,-0.77),
    Sinelon_Ocean(13,-0.75),
    Sinelon_Lava(14,-0.73),
    Sinelon_Forest(15,-0.71),
    BPM_Rainbow(16,-0.69),
    BPM_Party(17,-0.67),
    BPM_Ocean(18,-0.65),
    BPM_Lava(19,-0.63),
    BPM_Forest(20,-0.61),
    Fire_Medium(21,-0.59),
    Fire_Large(22,-0.57),
    Twinkels_Rainbow(23,-0.55),
    Twinkels_Party(24,-0.53),
    Twinkels_Ocean(25,-0.51),
    Twinkels_Lava(26,-0.49),
    Twinkels_Forest(27,-0.47),
    Color_Waves_Rainbow(28,-0.45),
    Color_Waves_Party(29,-0.43),
    Color_Waves_Ocean(30,-0.41),
    Color_Waves_Lava(31,-0.39),
    Color_Waves_Forest(32,-0.37),
    Larson_Red(33,-0.35),
    Larson_Gray(34,-0.33),
    Light_Chase_Red(35,-0.31),
    Light_Chase_Blue(36,-0.29),
    Light_Chase_Gray(37,-0.27),
    Heartbeat_Red(38,-0.25),
    Heartbeat_Blue(39,-0.23),
    Heartbeat_White(40,-0.21),
    Heartbeat_Gray(41,-0.19),
    Breath_Red(42,-0.17),
    Breath_Blue(43,-0.15),
    Breath_Gray(44,-0.13),
    Strobe_Red(45,-0.11),
    Strobe_Blue(46,-0.09),
    Strobe_Gold(47,-0.07),
    Strobe_White(48,-0.05),
    C1_Blend_To_Black(49,-0.03),
    C1_Larson_Scanner(50,-0.01),
    C1_Light_Chase(51,0.01),
    C1_Heartbeat_Slow(52,0.03),
    C1_Heartbeat_Medium(53,0.05),
    C1_Heartbeat_Fast(54,0.07),
    C1_Breath_Slow(55,0.09),
    C1_Breath_Fast(56,0.11),
    C1_Shot(57,0.13),
    C1_Strobe(58,0.15),
    C2_Blend_To_Black(59,0.17),
    C2_Larson_Scanner(60,0.19),
    C2_Light_Chase(61,0.21),
    C2_Heartbeat_Slow(62,0.23),
    C2_Heartbeat_Medium(63,0.25),
    C2_Heartbeat_Fast(64,0.27),
    C2_Breath_Slow(65,0.29),
    C2_Breath_Fast(66,0.31),
    C2_Shot(67,0.33),
    C2_Strobe(68,0.35),
    C1_C2_Sparkle_C1(69,0.37),
    C1_C2_Sparkle_C2(70,0.39),
    C1_C2_Gradient(71,0.41),
    C1_C2_BPM(72,0.43),
    C1_C2_Blend_C1(73,0.45),
    C1_C2_Blend_C2(74,0.47),
    C1_C2_No_Blend(75,0.49),
    C1_C2_Twinkles(76,0.51),
    C1_C2_Waves(77,0.53),
    C1_C2_Sinelon(78,0.55),
    Hot_Pink(79,0.57),
    Dark_Red(80,0.59),
    Red(81,0.61),
    Red_Orange(82,0.63),
    Orange(83,0.65),
    Gold(84,0.67),
    Yellow(85,0.69),
    Lawn_Green(86,0.71),
    Lime(87,0.73),
    Dark_Green(88,0.75),
    Green(89,0.77),
    Blue_Green(90,0.79),
    Aqua(91,0.81),
    Sky_Blue(92,0.83),
    Dark_Blue(93,0.85),
    Blue(94,0.87),
    Blue_Violet(95,0.89),
    Violet(96,0.91),
    White(97,0.93),
    Gray(98,0.95),
    Dark_Gray(99,0.97),
    Black(100,0.99);

    private double value;
    private int id;
    private REVColour(int id, double value){
        this.id = id;
        this.value = value;
    }

    public double getValue(){
        return value;
    }

    public int getId(){
        return id;
    }

    public static REVColour getByID(int id){
        for (int i = 0; i <  REVColour.values().length; i++) {
            REVColour colour =  REVColour.values()[i];
            if(colour.id == id) return colour;
        }
        return REVColour.values()[0];
    }
}
