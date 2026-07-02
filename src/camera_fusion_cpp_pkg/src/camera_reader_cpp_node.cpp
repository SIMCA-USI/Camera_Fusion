/**
 * @file camera_reader_cpp_node.cpp
 * @brief Nodo de lectura de cámaras USB en C++ puro.
 *
 * Descubre automáticamente las cámaras V4L2 disponibles, abre cada una en
 * un hilo dedicado y publica la imagen AL INSTANTE (fusión por eventos,
 * latencia cero, sin temporizadores que causen aliasing o tirones).
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <vector>
#include <memory>
#include <chrono>

class CameraReaderCppNode : public rclcpp::Node
{
public:
    CameraReaderCppNode() : Node("camera_reader_cpp_node"), running_(true)
    {
        this->declare_parameter("fps", 30);
        this->declare_parameter("width", 1920);
        this->declare_parameter("height", 1080);

        fps_    = this->get_parameter("fps").as_int();
        width_  = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();

        discover_cameras();
    }

    ~CameraReaderCppNode()
    {
        running_ = false;
        for (auto& t : capture_threads_) {
            if (t.joinable()) t.join();
        }
    }

private:
    struct Camera {
        int         id;
        std::string name;
        std::string frame_id;
        std::unique_ptr<cv::VideoCapture> cap;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
    };

    int fps_, width_, height_;
    std::atomic<bool> running_;
    std::vector<std::shared_ptr<Camera>> cameras_;
    std::vector<std::thread> capture_threads_;

    void discover_cameras()
    {
        int cam_idx = 1;
        for (int i = 0; i < 10; ++i) {
            auto cap = std::make_unique<cv::VideoCapture>(i, cv::CAP_V4L2);
            if (!cap->isOpened()) continue;

            // Forzar MJPG para minimizar el ancho de banda USB
            cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            // Obligamos al hardware de la cámara a entregar formato panorámico 16:9
            cap->set(cv::CAP_PROP_FRAME_WIDTH,  width_);
            cap->set(cv::CAP_PROP_FRAME_HEIGHT, height_);
            cap->set(cv::CAP_PROP_FPS,          fps_);
            // Buffer de 1 frame: siempre entrega el fotograma más reciente
            cap->set(cv::CAP_PROP_BUFFERSIZE, 1);

            // Validar que produce frames reales con contenido visible
            cv::Mat frame;
            bool valid = false;
            for (int j = 0; j < 5; ++j) {
                if (cap->read(frame) && !frame.empty()) {
                    cv::Scalar s = cv::sum(frame);
                    if (s[0] + s[1] + s[2] > 0.0) {
                        valid = true;
                        break;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            if (!valid) {
                RCLCPP_DEBUG(this->get_logger(), "/dev/video%d descartado (sin frames validos)", i);
                cap->release();
                continue;
            }

            auto cam  = std::make_shared<Camera>();
            cam->id   = i;
            cam->name = "cam_" + std::to_string(cam_idx++);
            cam->frame_id = cam->name + "_link";
            cam->cap  = std::move(cap);
            // Publicar con QoS óptimo para streaming
            cam->pub  = this->create_publisher<sensor_msgs::msg::Image>(
                cam->name + "/image_raw", 10);

            RCLCPP_INFO(this->get_logger(),
                "Camara %s detectada en /dev/video%d -> %s/image_raw",
                cam->name.c_str(), i, cam->name.c_str());

            cameras_.push_back(cam);
            capture_threads_.emplace_back(&CameraReaderCppNode::capture_thread_func, this, cam);
        }

        if (cameras_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No se detectaron camaras validas en /dev/video0..9");
        } else {
            RCLCPP_INFO(this->get_logger(), "Total camaras inicializadas: %zu. Publicando a MAXIMA velocidad sin lags.", cameras_.size());
        }
    }

    /**
     * @brief Hilo dedicado puramente a extraer fotogramas del hardware USB.
     * Publica INMEDIATAMENTE tras capturar para garantizar latencia CERO y cero tirones.
     */
    void capture_thread_func(std::shared_ptr<Camera> cam)
    {
        cv::Mat frame;
        while (running_ && rclcpp::ok()) {
            if (!cam->cap->grab()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            if (!cam->cap->retrieve(frame) || frame.empty()) {
                continue;
            }

            // Publicación inmediata sin pasar por temporizadores (Zero Timer Aliasing)
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = this->now();
            msg->header.frame_id = cam->frame_id;
            msg->height = frame.rows;
            msg->width = frame.cols;
            msg->encoding = "bgr8";
            msg->is_bigendian = false;
            msg->step = frame.cols * frame.elemSize();

            size_t size = msg->step * frame.rows;
            msg->data.resize(size);
            memcpy(msg->data.data(), frame.data, size);

            cam->pub->publish(std::move(msg));
        }
        cam->cap->release();
        RCLCPP_INFO(this->get_logger(), "Hilo de captura V4L2 de %s finalizado.", cam->name.c_str());
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraReaderCppNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
