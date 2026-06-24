/**
 * @file camera_reader_cpp_node.cpp
 * @brief Nodo de lectura de cámaras USB en C++ puro.
 *
 * Descubre automáticamente las cámaras V4L2 disponibles, abre cada una en
 * un hilo dedicado de C++ (Multithreading real, sin Python GIL) y publica
 * las imágenes en ROS 2 usando MJPG y buffer mínimo (= 1 frame) para
 * eliminar lag de cola y tirones.
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
    /**
     * @brief Constructor. Declara parámetros, descubre cámaras y arranca hilos de captura.
     */
    CameraReaderCppNode() : Node("camera_reader_cpp_node"), running_(true)
    {
        this->declare_parameter("fps", 30);
        this->declare_parameter("width", 640);
        this->declare_parameter("height", 480);

        fps_    = this->get_parameter("fps").as_int();
        width_  = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();

        discover_cameras();
    }

    /**
     * @brief Destructor. Detiene los hilos de captura de forma segura.
     */
    ~CameraReaderCppNode()
    {
        running_ = false;
        for (auto& t : capture_threads_) {
            if (t.joinable()) t.join();
        }
    }

private:
    /**
     * @brief Estructura interna que agrupa los recursos de una cámara.
     *
     * Se usa un puntero al VideoCapture para evitar copias (cv::VideoCapture
     * no es copiable). El objeto vive en el heap durante toda la vida del nodo.
     */
    struct Camera {
        int         id;
        std::string name;
        std::string frame_id;
        std::unique_ptr<cv::VideoCapture> cap;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
        
        cv::Mat latest_frame;
        std::mutex frame_mutex;
    };

    int fps_, width_, height_;
    std::atomic<bool> running_;
    std::vector<std::shared_ptr<Camera>> cameras_;
    std::vector<std::thread> capture_threads_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    /**
     * @brief Escanea /dev/video0..9, abre las cámaras válidas y crea un hilo por cada una.
     */
    void discover_cameras()
    {
        int cam_idx = 1;
        for (int i = 0; i < 10; ++i) {
            auto cap = std::make_unique<cv::VideoCapture>(i, cv::CAP_V4L2);
            if (!cap->isOpened()) continue;

            // Forzar MJPG para minimizar el ancho de banda USB
            cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            cap->set(cv::CAP_PROP_FRAME_WIDTH,  width_);
            cap->set(cv::CAP_PROP_FRAME_HEIGHT, height_);
            cap->set(cv::CAP_PROP_FPS,          fps_);
            // Buffer de 1 frame: siempre entrega el fotograma más reciente
            cap->set(cv::CAP_PROP_BUFFERSIZE, 1);

            // Validar que produce frames reales con contenido visible (no negros ni metadata-only)
            cv::Mat frame;
            bool valid = false;
            for (int j = 0; j < 5; ++j) {
                if (cap->read(frame) && !frame.empty()) {
                    // Comprobar que el frame no es completamente negro
                    // (descarta cámaras integradas en modo metadata y nodos V4L2 secundarios)
                    cv::Scalar s = cv::sum(frame);
                    if (s[0] + s[1] + s[2] > 0.0) {
                        valid = true;
                        break;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            if (!valid) {
                RCLCPP_DEBUG(this->get_logger(), "/dev/video%d descartado (sin frames válidos)", i);
                cap->release();
                continue;
            }

            auto cam  = std::make_shared<Camera>();
            cam->id   = i;
            cam->name = "cam_" + std::to_string(cam_idx++);
            cam->frame_id = cam->name + "_link";
            cam->cap  = std::move(cap);   // Transferencia de propiedad, sin copia
            cam->pub  = this->create_publisher<sensor_msgs::msg::Image>(
                cam->name + "/image_raw", rclcpp::SensorDataQoS());

            RCLCPP_INFO(this->get_logger(),
                "Camara %s detectada en /dev/video%d -> %s/image_raw",
                cam->name.c_str(), i, cam->name.c_str());

            cameras_.push_back(cam);
            capture_threads_.emplace_back(&CameraReaderCppNode::capture_thread_func, this, cam);
        }

        if (cameras_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No se detectaron camaras validas en /dev/video0..9");
        } else {
            RCLCPP_INFO(this->get_logger(), "Total camaras inicializadas: %zu", cameras_.size());
            
            // Iniciar el temporizador de publicación a la frecuencia objetivo (30 Hz)
            auto period = std::chrono::milliseconds(1000 / fps_);
            publish_timer_ = this->create_wall_timer(
                period, std::bind(&CameraReaderCppNode::publish_timer_callback, this));
        }
    }

    /**
     * @brief Hilo dedicado puramente a extraer fotogramas del hardware USB.
     * Nunca se bloquea por el middleware de ROS.
     */
    void capture_thread_func(std::shared_ptr<Camera> cam)
    {
        cv::Mat frame;
        while (running_ && rclcpp::ok()) {
            // grab() + retrieve() es lo mismo que read() pero recomendado en bucles de alto rendimiento
            if (!cam->cap->grab()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            if (!cam->cap->retrieve(frame) || frame.empty()) {
                continue;
            }

            // Guardar el último fotograma de forma segura
            {
                std::lock_guard<std::mutex> lock(cam->frame_mutex);
                frame.copyTo(cam->latest_frame);
            }
        }
        cam->cap->release();
        RCLCPP_INFO(this->get_logger(), "Hilo de captura V4L2 de %s finalizado.", cam->name.c_str());
    }

    /**
     * @brief Temporizador de ROS que publica los últimos fotogramas a 30 FPS exactos.
     */
    void publish_timer_callback()
    {
        for (auto& cam : cameras_) {
            cv::Mat frame_to_publish;
            {
                std::lock_guard<std::mutex> lock(cam->frame_mutex);
                if (cam->latest_frame.empty()) continue;
                // Clonamos para soltar el mutex rapido
                frame_to_publish = cam->latest_frame.clone();
            }

            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = this->now();
            msg->header.frame_id = cam->frame_id;
            msg->height = frame_to_publish.rows;
            msg->width = frame_to_publish.cols;
            msg->encoding = "bgr8";
            msg->is_bigendian = false;
            msg->step = frame_to_publish.cols * frame_to_publish.elemSize();

            size_t size = msg->step * frame_to_publish.rows;
            msg->data.resize(size);
            memcpy(msg->data.data(), frame_to_publish.data, size);

            cam->pub->publish(std::move(msg));
        }
    }
};

/**
 * @brief Punto de entrada del nodo de lectura de cámaras.
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraReaderCppNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
