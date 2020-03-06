package org.team401.robot2020.auto

enum class AutoMode {
    ROS, //Don't do anything.  This mode has been extensively tested for maximum performance.
    Shoot3, //Drive straight off the initiation line, then shoot 3 balls
    Trench6, //Drive and shoot 3 preload balls, collect 3 near balls from trench run, drive back, shoot
    HalfSneak, //Drive and shoot 3 preload balls, collect all 5 balls from trench run, drive back, shoot
    SneakyPete //Collect 2 near balls from opponent trench, drive back to goal, shoot all 5 balls
}