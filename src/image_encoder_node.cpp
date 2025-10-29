#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <ros_h264_streamer/h264_encoder.h>
#include <msg_all/CompressImage.h> // 自定义消息
#include <cv_bridge/cv_bridge.h>

class ImageEncoderNode {
public:
    ImageEncoderNode(ros::NodeHandle& nh){
        // 从参数服务器读取话题名称
        nh.param<std::string>("image_topic", image_topic_, "/camera/image_raw");
        nh.param<std::string>("encoded_topic", encoded_topic_, "/encoded_image");
        nh.param<int>("q_level", q_level_, 20);
        nh.param<int>("fps", fps_, 20);
        // 订阅图像话题
        image_sub_ = nh.subscribe(image_topic_, 1, &ImageEncoderNode::imageCallback, this);

        // 发布编码后的自定义消息
        encoded_pub_ = nh.advertise<msg_all::CompressImage>(encoded_topic_, 1);

        ROS_INFO("ImageEncoderNode initialized.");
    }

private:
    ros::Subscriber image_sub_;
    ros::Publisher encoded_pub_;
    std::string image_topic_;
    std::string encoded_topic_;
    int q_level_;
    int fps_;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // 编码图像
            ros_h264_streamer::H264Encoder encoder(msg->width, msg->height, q_level_, fps_, 1, msg->encoding);
            ros_h264_streamer::H264EncoderResult res = encoder.encode(msg);

            // 构造自定义消息
            msg_all::CompressImage encoded_msg;
            encoded_msg.header = msg->header;
            encoded_msg.frame_size = res.frame_size;
            encoded_msg.width = msg->width;
            encoded_msg.height = msg->height;
            encoded_msg.frame_data.data.insert(encoded_msg.frame_data.data.end(), res.frame_data, res.frame_data + res.frame_size);
            // 发布编码后的消息
            encoded_pub_.publish(encoded_msg);

            ROS_INFO("Encoded image published, size: %d bytes", res.frame_size);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to encode image: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_encoder_node");
    ros::NodeHandle nh("~");

    ImageEncoderNode encoder_node(nh);

    ros::spin();
    return 0;
}