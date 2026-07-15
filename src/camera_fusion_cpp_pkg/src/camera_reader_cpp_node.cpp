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

/**
 * @class CameraReaderCppNode
 * @brief Nodo para la lectura pura y publicación asíncrona de webcams USB.
 */
class CameraReaderCppNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor del nodo. Declara parámetros y lanza la búsqueda de hardware.
     */
    CameraReaderCppNode() : Node("camera_reader_cpp_node"), running_(true)
    {
        // ---------------------------------------------------------------------
        // 1. Declaración de parámetros ROS
        // ---------------------------------------------------------------------
        this->declare_parameter("fps", 30);
        this->declare_parameter("width", 1920);
        this->declare_parameter("height", 1080);

        fps_    = this->get_parameter("fps").as_int();
        width_  = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();

        // ---------------------------------------------------------------------
        // 2. Inicialización del Hardware
        // ---------------------------------------------------------------------
        discover_cameras();
    }

    /**
     * @brief Destructor. Garantiza la detención limpia de los hilos USB.
     */
    ~CameraReaderCppNode()
    {
        running_ = false;
        for (auto& t : capture_threads_) {
            if (t.joinable()) {
                t.join();
            }
        }
    }

private:
    // =========================================================================
    // Variables Miembro - Organizadas Lógicamente
    // =========================================================================

    /** @name Parámetros de Configuración ROS */
    ///@{
    int fps_;               ///< Tasa de captura en frames por segundo (V4L2)
    int width_;             ///< Resolución horizontal exigida a la cámara
    int height_;            ///< Resolución vertical exigida a la cámara
    ///@}

    /**
     * @struct Camera
     * @brief Contexto de ejecución de una única cámara física.
     */
    struct Camera {
        int id;                                         ///< Índice hardware /dev/videoX
        std::string name;                               ///< Identificador lógico ("cam_1", etc.)
        std::string frame_id;                           ///< Identificador del frame ROS ("cam_1_link")
        std::unique_ptr<cv::VideoCapture> cap;          ///< Interfaz de captura de OpenCV (V4L2)
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub; ///< Publicador topic individual
    };

    /** @name Hilos y Control de Hardware */
    ///@{
    std::atomic<bool> running_;                         ///< Bandera global para detener los bucles de los hilos
    std::vector<std::shared_ptr<Camera>> cameras_;      ///< Lista de cámaras activas y funcionales
    std::vector<std::thread> capture_threads_;          ///< Hilos dedicados a extraer vídeo sin bloquear el nodo
    ///@}

    // =========================================================================
    // Métodos Básicos de Hardware
    // =========================================================================

    /**
     * @brief Explora /dev/video0..9 para encontrar cámaras, configurarlas y asignarles hilos.
     */
    void discover_cameras()
    {
        int cam_idx = 1;
        for (int i = 0; i < 10; ++i) {
            auto cap = std::make_unique<cv::VideoCapture>(i, cv::CAP_V4L2);
            if (!cap->isOpened()) continue;

            // Forzar codec MJPG por hardware para minimizar el ancho de banda USB
            cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            // Obligamos al hardware de la cámara a entregar la resolución pedida (ej. 16:9)
            cap->set(cv::CAP_PROP_FRAME_WIDTH,  width_);
            cap->set(cv::CAP_PROP_FRAME_HEIGHT, height_);
            cap->set(cv::CAP_PROP_FPS,          fps_);
            // Buffer de 1 frame: evita que los fotogramas se acumulen con latencia
            cap->set(cv::CAP_PROP_BUFFERSIZE, 1);

            // Validar que la cámara produce fotogramas reales y no imágenes negras/vacías
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

            // Registrar cámara funcional
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
            
            // Iniciar hilo exclusivo de captura continua para esta cámara
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
     * @details Publica INMEDIATAMENTE tras capturar para garantizar latencia CERO y cero tirones,
     *          evitando el Timer Aliasing de un temporizador de ROS tradicional.
     * @param cam Puntero a la estructura de la cámara asociada a este hilo.
     */
    void capture_thread_func(std::shared_ptr<Camera> cam)
    {
        cv::Mat frame;
        while (running_ && rclcpp::ok()) {
            // grab() es rápido, saca la imagen de la memoria del chip USB
            if (!cam->cap->grab()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // retrieve() decodifica el MJPG a una matriz BGR (costoso en CPU)
            if (!cam->cap->retrieve(frame) || frame.empty()) {
                continue;
            }

            // Publicación inmediata sin pasar por temporizadores (Event-driven / Zero-Delay)
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

/**
 * @brief Punto de entrada (main) del proceso del nodo
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraReaderCppNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
