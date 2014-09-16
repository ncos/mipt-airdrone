#include <ros/ros.h>
#include <SFML/Graphics.hpp>
#include <geometry_msgs/Twist.h>

bool keyboard_control = false;
ros::Publisher  pub_vel;


void print_keyboard_help()
{
    ROS_INFO("\n\n=========================");
    ROS_INFO("USE WASD to control movement direction");
    ROS_INFO("USE Q E for left/right rotation");
    ROS_INFO("Z X for lower/higher altitude");
};

bool up       = false;
bool down     = false;
bool left     = false;
bool rght     = false;
bool rot_left = false;
bool rot_rght = false;
bool higher   = false;
bool lower    = false;


geometry_msgs::Twist keyboard_spin(sf::RenderWindow &window)
{
    const double v_mod = 0.5;
    const double a_mod = 0.5;

    geometry_msgs::Twist vel;

    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;

    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;

    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::KeyPressed) {
            if (event.key.code == sf::Keyboard::W) up       = true;
            if (event.key.code == sf::Keyboard::S) down     = true;
            if (event.key.code == sf::Keyboard::A) left     = true;
            if (event.key.code == sf::Keyboard::D) rght     = true;
            if (event.key.code == sf::Keyboard::Q) rot_left = true;
            if (event.key.code == sf::Keyboard::E) rot_rght = true;
            if (event.key.code == sf::Keyboard::X) higher   = true;
            if (event.key.code == sf::Keyboard::Z) lower    = true;
        }
        if (event.type == sf::Event::KeyReleased) {
            if (event.key.code == sf::Keyboard::W) up       = false;
            if (event.key.code == sf::Keyboard::S) down     = false;
            if (event.key.code == sf::Keyboard::A) left     = false;
            if (event.key.code == sf::Keyboard::D) rght     = false;
            if (event.key.code == sf::Keyboard::Q) rot_left = false;
            if (event.key.code == sf::Keyboard::E) rot_rght = false;
            if (event.key.code == sf::Keyboard::X) higher   = false;
            if (event.key.code == sf::Keyboard::Z) lower    = false;
        }

        if (up)       vel.linear.x  =  1;
        if (down)     vel.linear.x  = -1;
        if (left)     vel.linear.y  =  1;
        if (rght)     vel.linear.y  = -1;
        if (higher)   vel.linear.z  =  1;
        if (lower)    vel.linear.z  = -1;
        if (rot_left) vel.angular.z =  1;
        if (rot_rght) vel.angular.z = -1;

    }
    return vel;
};



int main( int argc, char** argv )
{
    ros::init(argc, argv, "velocity_server");
    ros::NodeHandle nh;
    ros::Rate loop_rate(60);
    std::string output_topic_kbrd = nh.resolveName("/cmd_vel_keyboard");
    pub_vel = nh.advertise<geometry_msgs::Twist > (output_topic_kbrd, 1);
    if (!nh.getParam("keyboard_control", keyboard_control)) keyboard_control = false;
    if (keyboard_control == false) {
        ROS_ERROR("The 'keyboard_control' parameter should be set to 'true' for this node to operate!");
        ros::shutdown();
    }

    print_keyboard_help();
    sf::RenderWindow window(sf::VideoMode(200, 200), "SFML copter control");

    while (ros::ok()) {
        pub_vel.publish(keyboard_spin(window));
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
};












