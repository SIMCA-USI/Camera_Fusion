/**
 * @file camera_fusion_cpp_node.cpp
 * @brief Nodo de fusión panorámica monolítico en C++ puro (Zero-Delay, Zero-DDS-overhead).
 * 
 * Este nodo se encarga de capturar imágenes desde dos cámaras USB en paralelo,
 * alinearlas utilizando matrices de calibración intrínsecas y homografía,
 * y aplicar una transición (Alpha Blending) para crear una imagen panorámica continua.
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <memory>
#include <mutex>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_srvs/srv/trigger.hpp>

/**
 * @class PanoramicFusionCppNode
 * @brief Nodo principal que hereda de rclcpp::Node para la fusión de cámaras.
 */
class PanoramicFusionCppNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor del nodo. Declara parámetros, carga calibraciones e inicia hilos.
     * @param options Opciones de inicialización de ROS 2.
     */
    PanoramicFusionCppNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
        : Node("panoramic_fusion_cpp_node", options), running_(true)
    {
        // ---------------------------------------------------------------------
        // 1. Declaración de parámetros ROS
        //    Valores por defecto: se sobreescriben desde config/params.yaml
        // ---------------------------------------------------------------------
        this->declare_parameter("fps",            30);
        this->declare_parameter("overlap_start",  475);
        this->declare_parameter("overlap_end",    525);
        this->declare_parameter("canvas_w",       1100);
        this->declare_parameter("canvas_h",       480);
        this->declare_parameter("cam_w",          640);
        this->declare_parameter("cam_h",          480);
        this->declare_parameter("interp",         static_cast<int>(cv::INTER_LINEAR));
        this->declare_parameter("indiv_resize_w", 640);
        this->declare_parameter("indiv_resize_h", 480);
        this->declare_parameter("margin_x",       30);
        this->declare_parameter("margin_y",       10);
        this->declare_parameter("swap_default",   true);

        // Directorio donde se buscan cam_1_calibration.yaml, cam_2_calibration.yaml y board_homography.yaml
        std::string default_config;
        try {
            default_config = ament_index_cpp::get_package_share_directory("camera_fusion_pkg") + "/config";
        } catch (const std::exception& e) {
            default_config = std::string(std::getenv("HOME")) + "/fusiones_ws/src/camera_fusion_pkg/config";
        }
        this->declare_parameter("config_dir", default_config);

        // --- Lectura de parámetros a variables miembro ---
        fps_            = this->get_parameter("fps").as_int();
        overlap_start_  = this->get_parameter("overlap_start").as_int();
        overlap_end_    = this->get_parameter("overlap_end").as_int();
        canvas_w_       = this->get_parameter("canvas_w").as_int();
        canvas_h_       = this->get_parameter("canvas_h").as_int();
        cam_w_          = this->get_parameter("cam_w").as_int();
        cam_h_          = this->get_parameter("cam_h").as_int();
        interp_         = this->get_parameter("interp").as_int();
        indiv_resize_w_ = this->get_parameter("indiv_resize_w").as_int();
        indiv_resize_h_ = this->get_parameter("indiv_resize_h").as_int();
        int margin_x    = this->get_parameter("margin_x").as_int();
        int margin_y    = this->get_parameter("margin_y").as_int();
        std::string config_dir = this->get_parameter("config_dir").as_string();


        // ---------------------------------------------------------------------
        // 2. Carga de Datos de Calibración
        // ---------------------------------------------------------------------
        load_intrinsics(config_dir + "/cam_1_calibration.yaml", K1_, D1_);
        load_intrinsics(config_dir + "/cam_2_calibration.yaml", K2_, D2_);
        load_homography(config_dir + "/board_homography.yaml", H_);

        cv::Size size_cam(cam_w_, cam_h_);
        cv::Size size_cnvs(canvas_w_, canvas_h_);

        // ---------------------------------------------------------------------
        // 3. Preparación de Mapas de Deformación (Warping Maps)
        // ---------------------------------------------------------------------
        // Cam 1: Solo aplica corrección de distorsión (K1, D1)
        cv::initUndistortRectifyMap(K1_, D1_, cv::Mat(), K1_, size_cam, CV_16SC2, map1x_, map1y_);

        // Cam 2: Corrección de distorsión + Transformación de perspectiva (Homografía H_)
        cv::Mat m2x, m2y;
        /* cv::initUndistortRectifyMap genera los mapas de deformación. Parámetros:
         * 1. K2_      : Matriz de cámara intrínseca (Focal y Centro óptico).
         * 2. D2_      : Coeficientes de distorsión de la lente.
         * 3. cv::Mat(): Matriz de rectificación (Vacía porque no es estéreo estándar).
         * 4. K2_      : Matriz de cámara objetivo (Mantenemos K2 para no escalar).
         * 5. size_cam : Tamaño de la imagen original (640x480).
         * 6. CV_32FC1 : Tipo de dato del mapa (Float de 32 bits, 1 canal).
         * 7. m2x, m2y : Matrices de salida (Mapas de coordenadas X e Y).
         */
        cv::initUndistortRectifyMap(K2_, D2_, cv::Mat(), K2_, size_cam, CV_32FC1, m2x, m2y);
        
        // Creación de una máscara para eliminar anomalías fuera del rango de la homografía
        cv::Mat cam_mask = cv::Mat::ones(cam_h_, cam_w_, CV_8UC1) * 255;
        cv::Mat warped_mask;
        cv::warpPerspective(cam_mask, warped_mask, H_, size_cnvs, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
        cv::threshold(warped_mask, warped_mask, 254, 255, cv::THRESH_BINARY);

        cv::warpPerspective(m2x, m2x, H_, size_cnvs, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(-1.0));
        cv::warpPerspective(m2y, m2y, H_, size_cnvs, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(-1.0));
        
        m2x.setTo(-1.0, warped_mask == 0);
        m2y.setTo(-1.0, warped_mask == 0);
        cv::convertMaps(m2x, m2y, map2x_, map2y_, CV_16SC2);

        // ---------------------------------------------------------------------
        // 4. Cálculo del Rectángulo de Recorte Final (Bounding Box)
        // ---------------------------------------------------------------------
        cv::Mat white = cv::Mat::ones(cam_h_, cam_w_, CV_8UC1) * 255;
        cv::Mat mask1, mask2;
        cv::remap(white, mask1, map1x_, map1y_, cv::INTER_NEAREST);
        cv::remap(white, mask2, map2x_, map2y_, cv::INTER_NEAREST);
        
        cv::Mat combined_mask = cv::Mat::zeros(canvas_h_, canvas_w_, CV_8UC1);
        cv::Mat roi_mask1(combined_mask, cv::Rect(0, 0, cam_w_, cam_h_));
        mask1.copyTo(roi_mask1);
        cv::bitwise_or(combined_mask, mask2, combined_mask);
        
        cv::Rect bbox = cv::boundingRect(combined_mask);

        // Limita el borde derecho al área válidamente proyectada de la cámara 2
        cv::Rect cam2_valid_bbox = cv::boundingRect(warped_mask);
        int right_bound = cam2_valid_bbox.x + cam2_valid_bbox.width;
        if (right_bound < bbox.x + bbox.width) {
            bbox.width = right_bound - bbox.x;
        }

        // Calcula coordenadas finales considerando los márgenes de seguridad
        crop_x_ = std::min(bbox.x + margin_x, canvas_w_ - 1);
        crop_y_ = std::min(bbox.y + margin_y, canvas_h_ - 1);
        crop_w_ = std::max(10, bbox.width - 2 * margin_x);
        crop_h_ = std::max(10, bbox.height - 2 * margin_y);

        RCLCPP_INFO(this->get_logger(),
            "Crop calculado → x=%d y=%d w=%d h=%d | cam2 right bound=%d",
            crop_x_, crop_y_, crop_w_, crop_h_, right_bound);

        // ---------------------------------------------------------------------
        // 5. Configuración de la Ventana de Difuminado (Alpha Blending)
        // ---------------------------------------------------------------------
        blend_s_ = std::max(0, std::min(overlap_start_, cam_w_ - 1));
        blend_e_ = std::max(blend_s_ + 1, std::min(overlap_end_, cam_w_));
        int bw = blend_e_ - blend_s_;
        
        // Array pre-calculado de pesos para transición lineal (ahorra CPU en bucle)
        alpha_1d_.resize(bw);
        for (int x = 0; x < bw; ++x) {
            alpha_1d_[x] = static_cast<uint16_t>(std::round(256.0 - (256.0 * x / (bw - 1))));
        }

        // ---------------------------------------------------------------------
        // 6. Preparación de Matrices en Memoria y Entidades ROS
        // ---------------------------------------------------------------------
        req1_ = cv::Rect(0, 0, blend_e_, cam_h_);
        req2_ = cv::Rect(blend_s_, 0, canvas_w_ - blend_s_, canvas_h_);
        
        out1_req_ = cv::Mat::zeros(req1_.height, req1_.width, CV_8UC3);
        out2_req_ = cv::Mat::zeros(req2_.height, req2_.width, CV_8UC3);
        canvas_.create(canvas_h_, canvas_w_, CV_8UC3);

        cv::setNumThreads(2);

        rclcpp::QoS qos(5);
        qos.reliable();
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/carrito/followme/video_frames", qos);

        // CameraInfo en modo TRANSIENT_LOCAL (llega a nuevos suscriptores automáticamente)
        rclcpp::QoS qos_info(1);
        qos_info.reliable();
        qos_info.transient_local();
        cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/carrito/followme/camera_info", qos_info);

        // Construye y publica el objeto CameraInfo panorámico una sola vez
        build_camera_info();

        swap_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/camera_fusion/swap_cameras",
            std::bind(&PanoramicFusionCppNode::swap_cameras_callback, this, std::placeholders::_1, std::placeholders::_2));

        // ---------------------------------------------------------------------
        // 7. Exploración e inicialización del Hardware USB
        // ---------------------------------------------------------------------
        discover_cameras();
    }

    /**
     * @brief Destructor del nodo. Detiene limpiamente los hilos concurrentes.
     */
    ~PanoramicFusionCppNode()
    {
        running_ = false;
        fusion_cv_.notify_all();
        if (fusion_thread_.joinable()) fusion_thread_.join();
        for (auto& t : capture_threads_) {
            if (t.joinable()) t.join();
        }
    }

private:
    // =========================================================================
    // Variables Miembro - Organizadas Lógicamente
    // =========================================================================

    /** @name Parámetros de Configuración ROS */
    ///@{
    int fps_;               ///< Tasa de captura en frames por segundo
    int overlap_start_;     ///< Píxel X de inicio del difuminado
    int overlap_end_;       ///< Píxel X de fin del difuminado
    int canvas_w_;          ///< Ancho total del lienzo de trabajo matemático
    int canvas_h_;          ///< Alto total del lienzo de trabajo matemático
    int cam_w_;             ///< Resolución X capturada físicamente (V4L2)
    int cam_h_;             ///< Resolución Y capturada físicamente (V4L2)
    int interp_;            ///< Modo de interpolación CV
    int indiv_resize_w_;    ///< Escala X para las publicaciones individuales
    int indiv_resize_h_;    ///< Escala Y para las publicaciones individuales
    int crop_x_;            ///< X inicial del rectángulo útil sin bordes negros
    int crop_y_;            ///< Y inicial del rectángulo útil sin bordes negros
    int crop_w_;            ///< Ancho de la panorámica resultante y publicada
    int crop_h_;            ///< Alto de la panorámica resultante y publicada
    int blend_s_;           ///< Variable auxiliar (inicio real de mezcla)
    int blend_e_;           ///< Variable auxiliar (fin real de mezcla)
    ///@}

    /** @name Variables Matemáticas y de Calibración */
    ///@{
    cv::Mat K1_, D1_;       ///< Matriz intrínseca y de distorsión (Cámara 1)
    cv::Mat K2_, D2_;       ///< Matriz intrínseca y de distorsión (Cámara 2)
    cv::Mat H_;             ///< Matriz de homografía entre ambas perspectivas
    cv::Mat map1x_, map1y_; ///< Array de desplazamientos en X/Y para remap de Cam 1
    cv::Mat map2x_, map2y_; ///< Array de desplazamientos en X/Y para remap de Cam 2
    std::vector<uint16_t> alpha_1d_; ///< Tabla de búsqueda rápida para difuminado
    ///@}

    /** @name Memoria de Trabajo (Para evitar re-allocations continuos) */
    ///@{
    cv::Mat out1_req_;      ///< Fragmento mapeado de la cámara 1
    cv::Mat out2_req_;      ///< Fragmento mapeado de la cámara 2
    cv::Mat canvas_;        ///< Lienzo grande donde se escribe la composición
    cv::Rect req1_;         ///< ROI activo de la cámara 1
    cv::Rect req2_;         ///< ROI activo de la cámara 2
    ///@}

    /**
     * @struct Camera
     * @brief Contexto de ejecución y hardware de una cámara individual.
     */
    struct Camera {
        int id;                                         ///< Índice hardware /dev/videoX
        std::string name;                               ///< Identificador lógico ("cam_1", etc.)
        std::unique_ptr<cv::VideoCapture> cap;          ///< Interfaz de captura de OpenCV
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub; ///< Publicador topic individual
        cv::Mat latest_frame;                           ///< Última imagen extraída
        std::mutex frame_mutex;                         ///< Cerrojo para acceso concurrente al frame
        std::chrono::high_resolution_clock::time_point last_pub_time;
        std::chrono::high_resolution_clock::time_point last_log_time;
        std::atomic<bool> is_fresh{false};              ///< Avisa si el fotograma ya fue procesado
    };

    /** @name Hilos y Sincronización Hardware-Software */
    ///@{
    std::atomic<bool> running_;                         ///< Flag global de ejecución limpia
    std::vector<std::shared_ptr<Camera>> cameras_;      ///< Arreglo de cámaras instanciadas
    std::vector<std::thread> capture_threads_;          ///< Pool de hilos de lectura USB
    std::thread fusion_thread_;                         ///< Hilo trabajador central de fusión
    std::mutex fusion_mutex_;                           ///< Cerrojo del ciclo principal
    std::condition_variable fusion_cv_;                 ///< Variable de condición (despertador de fusión)
    bool new_frame_ready_ = false;                      ///< Marca para la condición de despertar
    ///@}

    /** @name Elementos Centrales ROS 2 */
    ///@{
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;                 ///< Topic: video_frames (Panorámico)
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;   ///< Topic: camera_info (Metadatos 3D)
    sensor_msgs::msg::CameraInfo cam_info_msg_;                                 ///< Caché del CameraInfo
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr swap_srv_;               ///< Servicio: swap_cameras
    ///@}

    // =========================================================================
    // Métodos Básicos y Callbacks
    // =========================================================================

    /**
     * @brief Genera el modelo matemático 3D final (CameraInfo) de la panorámica resultante.
     * @details Se basa en la cámara 1 (undistorsionada) desplazando el centro óptico
     * por las coordenadas de recorte aplicadas al lienzo.
     */
    void build_camera_info()
    {
        cam_info_msg_.header.frame_id = "panoramic_link";
        cam_info_msg_.width  = static_cast<uint32_t>(crop_w_);
        cam_info_msg_.height = static_cast<uint32_t>(crop_h_);
        cam_info_msg_.distortion_model = "plumb_bob";

        // K1_ = [fx, 0, cx; 0, fy, cy; 0, 0, 1]  (CV_64F)
        double fx = K1_.at<double>(0, 0);
        double fy = K1_.at<double>(1, 1);
        // El centro óptico se desplaza restando el offset del recorte (crop)
        double cx = K1_.at<double>(0, 2) - static_cast<double>(crop_x_);
        double cy = K1_.at<double>(1, 2) - static_cast<double>(crop_y_);

        // D = ceros (porque la imagen de salida es perfecta y ortogonal)
        cam_info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0};

        // K (3x3 row-major)
        cam_info_msg_.k = {
            fx,  0.0,  cx,
            0.0,  fy,  cy,
            0.0, 0.0, 1.0
        };

        // R = identidad (sin rotación extra)
        cam_info_msg_.r = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };

        // P (3x4 row-major, sin traslación estéreo)
        cam_info_msg_.p = {
            fx,  0.0,  cx, 0.0,
            0.0,  fy,  cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        };

        cam_info_msg_.header.stamp = this->now();
        cam_info_pub_->publish(cam_info_msg_);

        RCLCPP_INFO(this->get_logger(),
            "CameraInfo panorámica: %dx%d | fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
            crop_w_, crop_h_, fx, fy, cx, cy);
    }

    /**
     * @brief Callback del servicio Trigger. Intercambia lógicamente las cámaras si están cruzadas físicamente.
     */
    bool swap_cameras_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        (void)req;
        std::lock_guard<std::mutex> lock(fusion_mutex_);
        if (cameras_.size() >= 2) {
            std::swap(cameras_[0], cameras_[1]);
            res->success = true;
            res->message = "Camaras intercambiadas correctamente.";
            RCLCPP_INFO(this->get_logger(), "Camaras intercambiadas por servicio GUI.");
        } else {
            res->success = false;
            res->message = "No hay suficientes camaras conectadas.";
        }
        return true;
    }

    /**
     * @brief Carga intrínsecos de OpenCV desde YAML. Si falla, usa Identidad y Ceros.
     */
    void load_intrinsics(const std::string& path, cv::Mat& K, cv::Mat& D) {
        try {
            YAML::Node config = YAML::LoadFile(path);
            std::vector<double> k_data = config["camera_matrix"]["data"].as<std::vector<double>>();
            std::vector<double> d_data = config["distortion_coefficients"]["data"].as<std::vector<double>>();
            K = cv::Mat(3, 3, CV_64F, k_data.data()).clone();
            D = cv::Mat(1, 5, CV_64F, d_data.data()).clone();
        } catch (const std::exception&) {
            K = cv::Mat::eye(3, 3, CV_64F);
            D = cv::Mat::zeros(1, 5, CV_64F);
            RCLCPP_WARN(this->get_logger(), "No se pudo cargar %s. Usando identidad.", path.c_str());
        }
    }

    /**
     * @brief Carga Homografía desde YAML. Si falla, usa Identidad (fusión plana errónea).
     */
    void load_homography(const std::string& path, cv::Mat& H) {
        try {
            YAML::Node config = YAML::LoadFile(path);
            std::vector<double> h_data = config["homography_matrix"]["data"].as<std::vector<double>>();
            H = cv::Mat(3, 3, CV_64F, h_data.data()).clone();
        } catch (const std::exception&) {
            H = cv::Mat::eye(3, 3, CV_64F);
            RCLCPP_WARN(this->get_logger(), "No se pudo cargar homografia. Usando identidad.");
        }
    }

    /**
     * @brief Consulta el nombre del hardware USB en Linux para detectar webcams integradas.
     */
    std::string get_camera_name(int index) {
        std::ifstream file("/sys/class/video4linux/video" + std::to_string(index) + "/name");
        std::string name;
        if (file.is_open()) {
            std::getline(file, name);
        }
        return name;
    }

    // =========================================================================
    // Core Engine (Subprocesos de captura y procesado)
    // =========================================================================

    /**
     * @brief Escanea buses USB y asigna 1 hilo exclusivo por cada cámara encontrada.
     */
    void discover_cameras()
    {
        int cam_idx = 1;
        for (int i = 0; i < 10; ++i) {
            if (cam_idx > 2) break; // Límite lógico de 2 cámaras (panorámica binocular)

            std::string hw_name = get_camera_name(i);
            // Salta cámaras inútiles como las de ordenadores portátiles de desarrollo
            if (hw_name.find("Integrated") != std::string::npos || hw_name.empty()) {
                continue; 
            }

            auto cap = std::make_unique<cv::VideoCapture>(i, cv::CAP_V4L2);
            if (!cap->isOpened()) continue;

            cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            cap->set(cv::CAP_PROP_FRAME_WIDTH,  cam_w_);
            cap->set(cv::CAP_PROP_FRAME_HEIGHT, cam_h_);
            cap->set(cv::CAP_PROP_FPS,          fps_);
            cap->set(cv::CAP_PROP_BUFFERSIZE, 1); // Evitar retención / latencia

            cv::Mat frame;
            bool valid = false;
            // Calentamiento del sensor y validación real de los píxeles
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
                cap->release();
                continue;
            }

            auto cam  = std::make_shared<Camera>();
            cam->id   = i;
            cam->name = "cam_" + std::to_string(cam_idx++);
            rclcpp::QoS qos_cam(5);
            qos_cam.reliable();
            cam->cap  = std::move(cap);
            cam->pub  = this->create_publisher<sensor_msgs::msg::Image>(
                cam->name + "/image_raw", qos_cam);

            RCLCPP_INFO(this->get_logger(), "Cámara %s descubierta en /dev/video%d", cam->name.c_str(), i);

            cameras_.push_back(cam);
            capture_threads_.emplace_back(&PanoramicFusionCppNode::capture_thread_func, this, cam);
        }

        if (cameras_.size() == 0) {
            RCLCPP_ERROR(this->get_logger(), "No se encontraron cámaras válidas.");
        } else if (cameras_.size() == 1) {
            RCLCPP_INFO(this->get_logger(), "[Modo 1 cámara] Iniciando publicación directa con undistort.");
            build_camera_info_single();
            fusion_thread_ = std::thread(&PanoramicFusionCppNode::single_cam_worker, this);
        } else {
            if (this->get_parameter("swap_default").as_bool()) {
                std::swap(cameras_[0], cameras_[1]);
                RCLCPP_INFO(this->get_logger(), "Cámaras intercambiadas automáticamente (swap_default=true).");
            }
            RCLCPP_INFO(this->get_logger(), "[Modo 2 cámaras] Iniciando fusión panorámica (Hardware-Synced).");
            fusion_thread_ = std::thread(&PanoramicFusionCppNode::fusion_worker, this);
        }
    }

    /**
     * @brief Tarea asíncrona dedicada por hardware a arrancar imágenes del bus USB.
     * @param cam Puntero inteligente al contexto de la cámara asignada a este hilo.
     */
    void capture_thread_func(std::shared_ptr<Camera> cam)
    {
        cv::Mat raw_frame;
        while (running_ && rclcpp::ok()) {
            // grab() extrae velozmente del buffer hardware sin decodificar JPEG
            if (!cam->cap->grab()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // retrieve() ejecuta la decodificación costosa
            if (!cam->cap->retrieve(raw_frame) || raw_frame.empty()) {
                continue;
            }

            // Copia profunda local: evitamos que la fusión congele al hilo USB
            cv::Mat new_frame = raw_frame.clone();

            {
                std::lock_guard<std::mutex> lock(cam->frame_mutex);
                std::swap(cam->latest_frame, new_frame);
            }
            cam->is_fresh = true;

            // Trigger Zero-Delay: La Cam 1 (Maestra) patea instantáneamente al hilo fusionador
            if (cam == cameras_[0]) {
                {
                    std::lock_guard<std::mutex> lock(fusion_mutex_);
                    new_frame_ready_ = true;
                }
                fusion_cv_.notify_one();
            }
        }
        cam->cap->release();
    }

    /**
     * @brief Prepara el mensaje de imagen individual y lo publica para el sistema secundario.
     */
    void publish_camera(std::shared_ptr<Camera> cam, const cv::Mat& raw_frame) {
        auto now = std::chrono::high_resolution_clock::now();
        if (cam->last_pub_time.time_since_epoch().count() > 0) {
            auto diff = std::chrono::duration<double, std::milli>(now - cam->last_pub_time).count();
            
            auto time_since_log = std::chrono::duration<double, std::milli>(now - cam->last_log_time).count();
            if (time_since_log >= 500.0) {
                RCLCPP_INFO(this->get_logger(), "[%s] Publish interval: %.2f ms", cam->name.c_str(), diff);
                cam->last_log_time = now;
            }
        }
        cam->last_pub_time = now;

        cv::Mat frame;
        if (indiv_resize_w_ > 0 && indiv_resize_h_ > 0 && 
           (indiv_resize_w_ != raw_frame.cols || indiv_resize_h_ != raw_frame.rows)) {
            cv::resize(raw_frame, frame, cv::Size(indiv_resize_w_, indiv_resize_h_), 0, 0, cv::INTER_LINEAR);
        } else {
            frame = raw_frame; // Zero-copy refcount increment
        }

        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = this->now();
        msg->header.frame_id = cam->name + "_link";
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

    /**
     * @brief Tarea que permanece bloqueada hasta que la cámara maestra la despierta.
     */
    void fusion_worker() {
        while (running_ && rclcpp::ok()) {
            std::unique_lock<std::mutex> lock(fusion_mutex_);
            fusion_cv_.wait(lock, [this]() { return new_frame_ready_ || !running_; });
            if (!running_) break;
            new_frame_ready_ = false;
            lock.unlock();

            do_fusion();
        }
    }

    /**
     * @brief Tarea para modo cámara única: aplica undistort y publica directamente en el topic panorámico.
     * @details Reutiliza el mismo mecanismo de condition_variable que el modo dual para despertar
     *          en cada fotograma nuevo, pero no aplica homografía ni alpha blending.
     */
    void single_cam_worker()
    {
        cv::Mat undistorted;
        while (running_ && rclcpp::ok()) {
            // Espera al trigger del capture_thread de la única cámara
            std::unique_lock<std::mutex> lock(fusion_mutex_);
            fusion_cv_.wait(lock, [this]() { return new_frame_ready_ || !running_; });
            if (!running_) break;
            new_frame_ready_ = false;
            lock.unlock();

            cv::Mat frame;
            {
                std::lock_guard<std::mutex> lock1(cameras_[0]->frame_mutex);
                if (cameras_[0]->latest_frame.empty()) continue;
                frame = cameras_[0]->latest_frame; // Referencia O(1), sin clone
            }
            cameras_[0]->is_fresh.exchange(false);

            // Undistort con los mapas ya precalculados de K1/D1 (sin homografía, sin blending)
            cv::remap(frame, undistorted, map1x_, map1y_, interp_);

            // Publica feed individual de la cámara (para grabación, SLAM, etc.)
            publish_camera(cameras_[0], frame);

            // Publica en el topic panorámico unificado para que el resto del sistema
            // no distinga si hay 1 o 2 cámaras
            auto out_msg            = std::make_unique<sensor_msgs::msg::Image>();
            auto stamp              = this->now();
            out_msg->header.stamp   = stamp;
            out_msg->header.frame_id = "panoramic_link";
            out_msg->height         = undistorted.rows;
            out_msg->width          = undistorted.cols;
            out_msg->encoding       = "bgr8";
            out_msg->is_bigendian   = false;
            out_msg->step           = undistorted.cols * 3;
            out_msg->data.resize(out_msg->step * out_msg->height);
            memcpy(out_msg->data.data(), undistorted.data, out_msg->data.size());
            pub_->publish(std::move(out_msg));

            cam_info_msg_.header.stamp = stamp;
            cam_info_pub_->publish(cam_info_msg_);
        }
    }

    /**
     * @brief Construye y publica el CameraInfo para el modo de cámara única.
     * @details Usa K1_ directamente, sin offset de recorte de fusión. La imagen
     *          de salida es cam_w_ x cam_h_ (dimensiones reales de captura undistorsionada).
     */
    void build_camera_info_single()
    {
        double fx = K1_.at<double>(0, 0);
        double fy = K1_.at<double>(1, 1);
        double cx = K1_.at<double>(0, 2);
        double cy = K1_.at<double>(1, 2);

        cam_info_msg_.header.frame_id  = "panoramic_link";
        cam_info_msg_.width            = static_cast<uint32_t>(cam_w_);
        cam_info_msg_.height           = static_cast<uint32_t>(cam_h_);
        cam_info_msg_.distortion_model = "plumb_bob";
        cam_info_msg_.d                = {0.0, 0.0, 0.0, 0.0, 0.0};
        cam_info_msg_.k = { fx,  0.0,  cx,
                             0.0,  fy,  cy,
                             0.0, 0.0, 1.0 };
        cam_info_msg_.r = { 1.0, 0.0, 0.0,
                             0.0, 1.0, 0.0,
                             0.0, 0.0, 1.0 };
        cam_info_msg_.p = { fx,  0.0,  cx, 0.0,
                             0.0,  fy,  cy, 0.0,
                             0.0, 0.0, 1.0, 0.0 };
        cam_info_msg_.header.stamp = this->now();
        cam_info_pub_->publish(cam_info_msg_);

        RCLCPP_INFO(this->get_logger(),
            "[CameraInfo única] %dx%d | fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
            cam_w_, cam_h_, fx, fy, cx, cy);
    }

    /**
     * @brief Consigue los últimos fotogramas con O(1) locks y coordina la lógica de fusión general.
     */
    void do_fusion()
    {
        if (cameras_.size() < 2) return;

        cv::Mat frame1, frame2;
        {
            std::lock_guard<std::mutex> lock1(cameras_[0]->frame_mutex);
            if (cameras_[0]->latest_frame.empty()) return;
            frame1 = cameras_[0]->latest_frame; // Referencia rápida O(1) (Sin clone)
        }
        {
            std::lock_guard<std::mutex> lock2(cameras_[1]->frame_mutex);
            if (cameras_[1]->latest_frame.empty()) return;
            frame2 = cameras_[1]->latest_frame;
        }

        bool fresh1 = cameras_[0]->is_fresh.exchange(false);
        bool fresh2 = cameras_[1]->is_fresh.exchange(false);

        // Alerta si los hilos USB no logran entregar fotogramas a la misma velocidad
        // if (!fresh1 || !fresh2) {
        //     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
        //         "TIRÓN DETECTADO: Usando frame repetido (cam_1 fresh: %d, cam_2 fresh: %d)", fresh1, fresh2);
        // }

        // Publica feeds sin fusionar para componentes como SLAM o grabación
        publish_camera(cameras_[0], frame1);
        publish_camera(cameras_[1], frame2);
        
        fuse(frame1, frame2);
    }

    /**
     * @brief Algoritmo matemático final: Mapea (deforma), copia, difumina, recorta y envía por red.
     */
    void fuse(const cv::Mat& cv_ptr1, const cv::Mat& cv_ptr2) {
        try {
            auto start_fuse = std::chrono::high_resolution_clock::now();
            
            // 1. Spatial Transformation (Remap)
            // Se realiza SOLO en las regiones ROI matemáticamente necesarias (gran ahorro de CPU)
            cv::remap(cv_ptr1, out1_req_, map1x_(req1_), map1y_(req1_), interp_);
            cv::remap(cv_ptr2, out2_req_, map2x_(req2_), map2y_(req2_), interp_);

            // 2. Copia directa de memoria
            // Pega las zonas que no solapan (Cámara 1 a la izquierda, Cámara 2 a la derecha)
            int bw = blend_e_ - blend_s_;
            out1_req_(cv::Rect(0, 0, blend_s_, cam_h_)).copyTo(canvas_(cv::Rect(0, 0, blend_s_, cam_h_)));
            
            int e_w = canvas_w_ - blend_e_;
            out2_req_(cv::Rect(bw, 0, e_w, canvas_h_)).copyTo(canvas_(cv::Rect(blend_e_, 0, e_w, canvas_h_)));

            // 3. CPU Cache-friendly Alpha Blending
            // Mezcla las zonas centrales usando un degradado (Transición suave sin bordes feos)
            cv::Mat blend1 = out1_req_(cv::Rect(blend_s_, 0, bw, cam_h_));
            cv::Mat blend2 = out2_req_(cv::Rect(0, 0, bw, canvas_h_));
            cv::Mat blend_dst = canvas_(cv::Rect(blend_s_, 0, bw, cam_h_));

            for (int y = 0; y < cam_h_; ++y) {
                const uchar* p1 = blend1.ptr<uchar>(y);
                const uchar* p2 = blend2.ptr<uchar>(y);
                uchar* pdst = blend_dst.ptr<uchar>(y);
                for (int x = 0; x < bw; ++x) {
                    uint16_t a1 = alpha_1d_[x];
                    uint16_t a2 = 256 - a1;
                    int idx = x * 3;
                    
                    // Sustitución de división (/256) por barrido de bits (>> 8) por optimización extrema
                    pdst[idx]   = (p1[idx]   * a1 + p2[idx]   * a2) >> 8;
                    pdst[idx+1] = (p1[idx+1] * a1 + p2[idx+1] * a2) >> 8;
                    pdst[idx+2] = (p1[idx+2] * a1 + p2[idx+2] * a2) >> 8;
                }
            }

            // 4. Recorte de márgenes
            // Se aísla y recorta la zona con información válida, ignorando huecos negros laterales
            cv::Mat cropped = canvas_(cv::Rect(crop_x_, crop_y_, crop_w_, crop_h_));
            
            // 5. Encapsulación y Publicación (ROS 2 Serializer)
            auto out_msg = std::make_unique<sensor_msgs::msg::Image>();
            out_msg->header.stamp = this->now();
            out_msg->header.frame_id = "panoramic_link";
            out_msg->height       = cropped.rows;
            out_msg->width        = cropped.cols;
            out_msg->encoding     = "bgr8";
            out_msg->is_bigendian = false;
            out_msg->step         = cropped.cols * 3;
            out_msg->data.resize(out_msg->step * out_msg->height);

            cv::Mat final_canvas(cropped.rows, cropped.cols, CV_8UC3, out_msg->data.data());
            cropped.copyTo(final_canvas);

            auto start_pub = std::chrono::high_resolution_clock::now();
            auto stamp = this->now();
            
            // Disparo sincrónico de ambos mensajes al bus ROS
            out_msg->header.stamp = stamp;
            pub_->publish(std::move(out_msg));

            cam_info_msg_.header.stamp = stamp;
            cam_info_pub_->publish(cam_info_msg_);

            auto end_pub = std::chrono::high_resolution_clock::now();

            auto duration_fuse = std::chrono::duration<double, std::milli>(start_pub - start_fuse).count();
            auto duration_pub = std::chrono::duration<double, std::milli>(end_pub - start_pub).count();
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "Fuse took: %.2f ms | Publish took: %.2f ms", duration_fuse, duration_pub);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Excepción en fusión: %s", e.what());
        }
    }
};

/**
 * @brief Punto de entrada (main) del proceso del nodo
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    
    // Configuración obligatoria para lograr el Zero-Copy si alguien se suscribe intra-proceso
    options.use_intra_process_comms(true);
    
    auto node = std::make_shared<PanoramicFusionCppNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
