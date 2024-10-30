#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <serial/serial.h>
#include <vector>
#include <cstring>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "battery_parser_node");
    ros::NodeHandle nh;
    ros::NodeHandle param_nh("~");

    // Read/set parameters
    std::string port;
    int refresh_rate_hz;
    param_nh.param("port", port, std::string("/dev/ttyUSB0"));
    param_nh.param("refresh_rate_hz", refresh_rate_hz, 1);

    ros::Publisher battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery_state", 10);
    ros::Rate loop_rate(refresh_rate_hz);

    // serial settings
    serial::Serial *serial_port;
    try
    {
        serial_port = new serial::Serial(port);
    }
    catch (...)
    {
        // TODO: add error message in exception and report
        ROS_ERROR("Unable to initalize battery serial port");
        return 1;
    }
    // serial_port.setPort(port);
    // serial_port.setBaudrate(9600);
    // serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    // serial_port.setTimeout(to);
    // serial_port.open();
    // if (!serial_port.isOpen())
    // {
    //     ROS_ERROR("Failed to open serial port!");
    // }

    while (ros::ok())
    {
        // 发送请求
        uint8_t request[] = {0xdd, 0xa5, 0x03, 0x00, 0xff, 0xfd, 0x77};
        serial_port->write(request, sizeof(request));

        // 读取数据
        std::vector<uint8_t> buffer(24);
        size_t bytes_read = serial_port->read(buffer.data(), buffer.size());

        if (bytes_read >= 24 && buffer[0] == 0xdd && buffer[1] == 0x03)
        {
            // decode
            uint8_t rsoc = buffer[23];                       // 电量
            uint16_t voltage = (buffer[4] << 8) | buffer[5]; // 总电压
            int16_t current = (buffer[6] << 8) | buffer[7];  // 电流

            // publish battery msg
            sensor_msgs::BatteryState battery_msg;
            battery_msg.header.stamp = ros::Time::now();
            battery_msg.percentage = rsoc / 100.0; // 转换为百分比
            battery_msg.voltage = voltage * 0.01;  // 转换为伏特
            battery_msg.current = current * 0.01;  // 转换为安培

            battery_pub.publish(battery_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    delete serial_port;
    return 0;
}