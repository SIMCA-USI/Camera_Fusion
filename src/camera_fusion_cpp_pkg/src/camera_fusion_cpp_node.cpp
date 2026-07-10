/**
 * @file camera_fusion_cpp_node.cpp
 * @brief Nodo de fusión panorámica monolítico en C++ puro (Zero-Delay, Zero-DDS-overhead).
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

class PanoramicFusionCppNode : public rclcpp::Node
{
public:
    PanoramicFusionCppNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
        : Node("panoramic_fusion_cpp_node", options), running_(true)
    {
        // 1. Declare and get parameters
        this->declare_parameter("fps", 30);
        this->declare_parameter("overlap_start", 400);
        this->declare_parameter("overlap_end", 600);
        this->declare_parameter("canvas_w", 1100);
        this->declare_parameter("canvas_h", 480);
        this->declare_parameter("cam_w", 640);
        this->declare_parameter("cam_h", 480);
        this->declare_parameter("interp", static_cast<int>(cv::INTER_LINEAR));
        this->declare_parameter("indiv_resize_w", 640);
        this->declare_parameter("indiv_resize_h", 480);
        
        std::string default_config;
        try {
            default_config = ament_index_cpp::get_package_share_directory("camera_fusion_pkg") + "/config";
        } catch (const std::exception& e) {
            default_config = std::string(std::getenv("HOME")) + "/fusiones_ws/src/camera_fusion_pkg/config";
        }
        
        this->declare_parameter("config_dir", default_config);
        this->declare_parameter("margin_x", 30);
        this->declare_parameter("margin_y", 10);

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

        // 2. Load Calibration Data
        load_intrinsics(config_dir + "/cam_1_calibration.yaml", K1_, D1_);
        load_intrinsics(config_dir + "/cam_2_calibration.yaml", K2_, D2_);
        load_homography(config_dir + "/board_homography.yaml", H_);

        cv::Size size_cam(cam_w_, cam_h_);
        cv::Size size_cnvs(canvas_w_, canvas_h_);

        // 3. Prepare Warping Maps
        cv::initUndistortRectifyMap(K1_, D1_, cv::Mat(), K1_, size_cam, CV_16SC2, map1x_, map1y_);

        cv::Mat m2x, m2y;
        cv::initUndistortRectifyMap(K2_, D2_, cv::Mat(), K2_, size_cam, CV_32FC1, m2x, m2y);
        
        cv::Mat cam_mask = cv::Mat::ones(cam_h_, cam_w_, CV_8UC1) * 255;
        cv::Mat warped_mask;
        cv::warpPerspective(cam_mask, warped_mask, H_, size_cnvs, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
        cv::threshold(warped_mask, warped_mask, 254, 255, cv::THRESH_BINARY);

        cv::warpPerspective(m2x, m2x, H_, size_cnvs, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(-1.0));
        cv::warpPerspective(m2y, m2y, H_, size_cnvs, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(-1.0));
        
        m2x.setTo(-1.0, warped_mask == 0);
        m2y.setTo(-1.0, warped_mask == 0);
        cv::convertMaps(m2x, m2y, map2x_, map2y_, CV_16SC2);

        // 4. Compute Bounding Box for Cropping
        cv::Mat white = cv::Mat::ones(cam_h_, cam_w_, CV_8UC1) * 255;
        cv::Mat mask1, mask2;
        cv::remap(white, mask1, map1x_, map1y_, cv::INTER_NEAREST);
        cv::remap(white, mask2, map2x_, map2y_, cv::INTER_NEAREST);
        
        cv::Mat combined_mask = cv::Mat::zeros(canvas_h_, canvas_w_, CV_8UC1);
        cv::Mat roi_mask1(combined_mask, cv::Rect(0, 0, cam_w_, cam_h_));
        mask1.copyTo(roi_mask1);
        cv::bitwise_or(combined_mask, mask2, combined_mask);
        
        cv::Rect bbox = cv::boundingRect(combined_mask);

        // Clamp right boundary to cam2's valid warped region.
        // combined_mask can include remap-boundary pixels that are technically
        // "valid" but map to black (outside cam2's FOV), producing the right
        // black column. warped_mask is thresholded at 254 so it only marks
        // pixels where the homography projects real cam2 content.
        cv::Rect cam2_valid_bbox = cv::boundingRect(warped_mask);
        int right_bound = cam2_valid_bbox.x + cam2_valid_bbox.width;
        if (right_bound < bbox.x + bbox.width) {
            bbox.width = right_bound - bbox.x;
        }

        crop_x_ = std::min(bbox.x + margin_x, canvas_w_ - 1);
        crop_y_ = std::min(bbox.y + margin_y, canvas_h_ - 1);
        crop_w_ = std::max(10, bbox.width - 2 * margin_x);
        crop_h_ = std::max(10, bbox.height - 2 * margin_y);

        RCLCPP_INFO(this->get_logger(),
            "Crop calculado → x=%d y=%d w=%d h=%d | cam2 right bound=%d",
            crop_x_, crop_y_, crop_w_, crop_h_, right_bound);

        // 5. Blending setup
        blend_s_ = std::max(0, std::min(overlap_start_, cam_w_ - 1));
        blend_e_ = std::max(blend_s_ + 1, std::min(overlap_end_, cam_w_));
        int bw = blend_e_ - blend_s_;
        
        alpha_1d_.resize(bw);
        for (int x = 0; x < bw; ++x) {
            alpha_1d_[x] = static_cast<uint16_t>(std::round(256.0 - (256.0 * x / (bw - 1))));
        }

        // 6. Output Canvases & Threading
        req1_ = cv::Rect(0, 0, blend_e_, cam_h_);
        req2_ = cv::Rect(blend_s_, 0, canvas_w_ - blend_s_, canvas_h_);
        
        out1_req_ = cv::Mat::zeros(req1_.height, req1_.width, CV_8UC3);
        out2_req_ = cv::Mat::zeros(req2_.height, req2_.width, CV_8UC3);
        canvas_.create(canvas_h_, canvas_w_, CV_8UC3);

        cv::setNumThreads(2);

        rclcpp::QoS qos(5);
        qos.reliable();

        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/carrito/followme/video_frames", qos);

        // CameraInfo: TRANSIENT_LOCAL = latched, cualquier suscriptor que se conecte
        // después recibirá el último mensaje inmediatamente.
        rclcpp::QoS qos_info(1);
        qos_info.reliable();
        qos_info.transient_local();
        cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/carrito/followme/camera_info", qos_info);

        // Construye la CameraInfo una sola vez (valores estáticos post-crop)
        build_camera_info();

        swap_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/camera_fusion/swap_cameras",
            std::bind(&PanoramicFusionCppNode::swap_cameras_callback, this, std::placeholders::_1, std::placeholders::_2));

        // 7. Initialize Cameras
        discover_cameras();
    }

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
    // ---- Configuration Parameters ----
    int fps_, overlap_start_, overlap_end_, canvas_w_, canvas_h_, cam_w_, cam_h_, interp_;
    int indiv_resize_w_, indiv_resize_h_;
    int crop_x_, crop_y_, crop_w_, crop_h_;
    int blend_s_, blend_e_;

    // ---- Calibration & Warping ----
    cv::Mat K1_, D1_, K2_, D2_, H_;
    cv::Mat map1x_, map1y_, map2x_, map2y_;
    std::vector<uint16_t> alpha_1d_;

    // ---- Working Memory (Avoid allocations in loop) ----
    cv::Mat out1_req_, out2_req_, canvas_;
    cv::Rect req1_, req2_;

    // ---- Hardware Capture & Threading ----
    struct Camera {
        int id;
        std::string name;
        std::unique_ptr<cv::VideoCapture> cap;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
        cv::Mat latest_frame;
        std::mutex frame_mutex;
        std::chrono::high_resolution_clock::time_point last_pub_time;
        std::chrono::high_resolution_clock::time_point last_log_time;
        std::atomic<bool> is_fresh{false};
    };

    std::atomic<bool> running_;
    std::vector<std::shared_ptr<Camera>> cameras_;
    std::vector<std::thread> capture_threads_;
    
    // ---- Hardware Synchronization ----
    std::thread fusion_thread_;
    std::mutex fusion_mutex_;
    std::condition_variable fusion_cv_;
    bool new_frame_ready_ = false;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;
    sensor_msgs::msg::CameraInfo cam_info_msg_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr swap_srv_;

    // =========================================================================
    // Core Methods
    // =========================================================================

    /**
     * Construye la sensor_msgs::CameraInfo para la imagen panorámica fusionada.
     *
     * La imagen fusionada ya está undistorsionada → D = zeros.
     * cam_1 se coloca directamente (solo undistorsión), así que su K intrínseca
     * es válida como aproximación global. El crop desplaza el origen de imagen:
     *   cx_new = cx - crop_x_
     *   cy_new = cy - crop_y_
     * fx y fy no varían (el crop no escala).
     *
     * Se llama una sola vez tras calcular crop_x_/y_/w_/h_.
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
        double cx = K1_.at<double>(0, 2) - static_cast<double>(crop_x_);
        double cy = K1_.at<double>(1, 2) - static_cast<double>(crop_y_);

        // D = zeros (ya undistorsionada)
        cam_info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0};

        // K (3x3 row-major)
        cam_info_msg_.k = {
            fx,  0.0,  cx,
            0.0,  fy,  cy,
            0.0, 0.0, 1.0
        };

        // R = identidad
        cam_info_msg_.r = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };

        // P (3x4 row-major, sin baseline)
        cam_info_msg_.p = {
            fx,  0.0,  cx, 0.0,
            0.0,  fy,  cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        };

        // Publica inmediatamente (TRANSIENT_LOCAL → cualquier suscriptor futuro la recibe)
        cam_info_msg_.header.stamp = this->now();
        cam_info_pub_->publish(cam_info_msg_);

        RCLCPP_INFO(this->get_logger(),
            "CameraInfo panorámica: %dx%d | fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
            crop_w_, crop_h_, fx, fy, cx, cy);
    }

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

    std::string get_camera_name(int index) {
        std::ifstream file("/sys/class/video4linux/video" + std::to_string(index) + "/name");
        std::string name;
        if (file.is_open()) {
            std::getline(file, name);
        }
        return name;
    }

    void discover_cameras()
    {
        int cam_idx = 1;
        for (int i = 0; i < 10; ++i) {
            if (cam_idx > 2) break; // Solo necesitamos 2 camaras

            std::string hw_name = get_camera_name(i);
            // Ignorar las cámaras web integradas del portatil
            if (hw_name.find("Integrated") != std::string::npos || hw_name.empty()) {
                continue; 
            }

            auto cap = std::make_unique<cv::VideoCapture>(i, cv::CAP_V4L2);
            if (!cap->isOpened()) continue;

            cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            cap->set(cv::CAP_PROP_FRAME_WIDTH,  cam_w_);
            cap->set(cv::CAP_PROP_FRAME_HEIGHT, cam_h_);
            cap->set(cv::CAP_PROP_FPS,          fps_);
            cap->set(cv::CAP_PROP_BUFFERSIZE, 1);

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

            RCLCPP_INFO(this->get_logger(), "Camara %s descubierta en /dev/video%d", cam->name.c_str(), i);

            cameras_.push_back(cam);
            capture_threads_.emplace_back(&PanoramicFusionCppNode::capture_thread_func, this, cam);
        }

        if (cameras_.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "No se encontraron 2 camaras validas.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Camaras enlazadas. Iniciando fusion directa (Hardware-Synced).");
            fusion_thread_ = std::thread(&PanoramicFusionCppNode::fusion_worker, this);
        }
    }

    void capture_thread_func(std::shared_ptr<Camera> cam)
    {
        cv::Mat raw_frame;
        while (running_ && rclcpp::ok()) {
            if (!cam->cap->grab()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            if (!cam->cap->retrieve(raw_frame) || raw_frame.empty()) {
                continue;
            }

            // Copia profunda fuera del hilo principal para no bloquear la fusion
            cv::Mat new_frame = raw_frame.clone();

            {
                std::lock_guard<std::mutex> lock(cam->frame_mutex);
                std::swap(cam->latest_frame, new_frame);
            }
            cam->is_fresh = true;

            // Si esta es la camara maestra (cam_1), dispara la fusion
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

    void do_fusion()
    {
        if (cameras_.size() < 2) return;

        cv::Mat frame1, frame2;
        {
            std::lock_guard<std::mutex> lock1(cameras_[0]->frame_mutex);
            if (cameras_[0]->latest_frame.empty()) return;
            frame1 = cameras_[0]->latest_frame; // Referencia rapida O(1)
        }
        {
            std::lock_guard<std::mutex> lock2(cameras_[1]->frame_mutex);
            if (cameras_[1]->latest_frame.empty()) return;
            frame2 = cameras_[1]->latest_frame;
        }

        bool fresh1 = cameras_[0]->is_fresh.exchange(false);
        bool fresh2 = cameras_[1]->is_fresh.exchange(false);

        if (!fresh1 || !fresh2) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "TIRON DETECTADO: Usando frame repetido (cam_1 fresh: %d, cam_2 fresh: %d)", fresh1, fresh2);
        }

        publish_camera(cameras_[0], frame1);
        publish_camera(cameras_[1], frame2);
        fuse(frame1, frame2);
    }

    void fuse(const cv::Mat& cv_ptr1, const cv::Mat& cv_ptr2) {
        try {
            auto start_fuse = std::chrono::high_resolution_clock::now();
            
            // 1. Spatial Transformation (Remap) ONLY on needed regions
            cv::remap(cv_ptr1, out1_req_, map1x_(req1_), map1y_(req1_), interp_);
            cv::remap(cv_ptr2, out2_req_, map2x_(req2_), map2y_(req2_), interp_);

            // 2. Direct Memory Copy for non-overlapping borders
            int bw = blend_e_ - blend_s_;
            out1_req_(cv::Rect(0, 0, blend_s_, cam_h_)).copyTo(canvas_(cv::Rect(0, 0, blend_s_, cam_h_)));
            
            int e_w = canvas_w_ - blend_e_;
            out2_req_(cv::Rect(bw, 0, e_w, canvas_h_)).copyTo(canvas_(cv::Rect(blend_e_, 0, e_w, canvas_h_)));

            // 3. Fast In-place CPU Cache-friendly Alpha Blending
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
                    pdst[idx]   = (p1[idx]   * a1 + p2[idx]   * a2) >> 8;
                    pdst[idx+1] = (p1[idx+1] * a1 + p2[idx+1] * a2) >> 8;
                    pdst[idx+2] = (p1[idx+2] * a1 + p2[idx+2] * a2) >> 8;
                }
            }

            cv::Mat cropped = canvas_(cv::Rect(crop_x_, crop_y_, crop_w_, crop_h_));
            
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
            out_msg->header.stamp = stamp;
            pub_->publish(std::move(out_msg));

            // Publica CameraInfo con el mismo timestamp que el frame
            cam_info_msg_.header.stamp = stamp;
            cam_info_pub_->publish(cam_info_msg_);

            auto end_pub = std::chrono::high_resolution_clock::now();

            auto duration_fuse = std::chrono::duration<double, std::milli>(start_pub - start_fuse).count();
            auto duration_pub = std::chrono::duration<double, std::milli>(end_pub - start_pub).count();
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "Fuse took: %.2f ms | Publish took: %.2f ms", duration_fuse, duration_pub);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Excepcion en fusion: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    auto node = std::make_shared<PanoramicFusionCppNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
