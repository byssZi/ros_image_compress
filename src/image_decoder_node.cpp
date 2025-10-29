#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <ros_h264_streamer/h264_decoder.h>
#include <msg_all/CompressImage.h> // 自定义消息

class ImageDecoderNode {
public:
    ImageDecoderNode(ros::NodeHandle& nh){
        // 从参数服务器读取话题名称
        nh.param<std::string>("image_restored_topic", image_restored_topic_, "/camera/image_raw_restore");
        nh.param<std::string>("encoded_topic", encoded_topic_, "/encoded_image");
        
        // 订阅编码后的自定义消息
        encoded_sub_ = nh.subscribe(encoded_topic_, 1, &ImageDecoderNode::encodedCallback, this);

        // 发布解码后的图像话题
        image_pub_ = nh.advertise<sensor_msgs::Image>(image_restored_topic_, 1);

        ROS_INFO("ImageDecoderNode initialized.");
    }

private:
    ros::Subscriber encoded_sub_;
    ros::Publisher image_pub_;
    std::string image_restored_topic_;
    std::string encoded_topic_;

    void encodedCallback(const msg_all::CompressImagePtr& msg) {
        try {
            
            ros_h264_streamer::H264Decoder decoder(msg->width, msg->height);
            // 解码图像
            sensor_msgs::ImagePtr decoded_image(new sensor_msgs::Image);
            uint8_t* data_ptr = msg->frame_data.data.data();
            int len = decoder.decode(msg->frame_size, data_ptr, decoded_image);
            if (len > 0) {
                // 发布解码后的图像
                image_pub_.publish(decoded_image);
                ROS_INFO("Decoded image published, size: %dx%d", decoded_image->width, decoded_image->height);
            } else {
                ROS_WARN("Failed to decode image.");
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to decode image: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_decoder_node");
    ros::NodeHandle nh("~");

    ImageDecoderNode decoder_node(nh);

    ros::spin();
    return 0;
}