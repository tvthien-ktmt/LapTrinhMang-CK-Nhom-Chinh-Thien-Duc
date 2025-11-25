#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <atomic>
#include <mutex>
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <vector>

using namespace mavsdk;
using namespace std::chrono_literals;

// ==================== HÀM NHẬN PHÍM KHÔNG BLOCK ====================
void setNonBlocking(bool enable) {
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);

    if (enable) {
        ttystate.c_lflag &= ~ICANON; // Tắt canonical mode
        ttystate.c_lflag &= ~ECHO;   // Tắt echo
        ttystate.c_cc[VMIN] = 0;
        ttystate.c_cc[VTIME] = 0;
    } else {
        ttystate.c_lflag |= ICANON;
        ttystate.c_lflag |= ECHO;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

int kbhit() {
    struct timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
}

char getch() {
    char c;
    if (read(STDIN_FILENO, &c, 1) < 0) return 0;
    return c;
}

// ==================== DRONE MANAGER ====================
class DroneManager {
private:
    std::shared_ptr<System> system_;
    std::shared_ptr<Telemetry> telemetry_;
    std::shared_ptr<Action> action_;

    std::atomic<bool> monitoring_{false};
    std::atomic<bool> mission_running_{false};
    std::atomic<bool> manual_mode_{false};

    std::thread monitor_thread_;
    std::thread mission_thread_;
    std::mutex status_mutex_;
    std::mutex mission_mutex_;

    std::string mission_name_ = "None";

    Telemetry::Position current_pos_{};
    Telemetry::VelocityNed current_vel_{};
    Telemetry::Battery current_battery_{};
    
public:
    DroneManager(std::shared_ptr<System> system)
        : system_(system)
    {
        telemetry_ = std::make_shared<Telemetry>(system_);
        action_ = std::make_shared<Action>(system_);
    }

    ~DroneManager() {
        stop_monitoring();
        stop_mission();
    }

    void set_mission_name(const std::string &name) {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        mission_name_ = name;
    }

    std::string get_mission_name() {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        return mission_name_;
    }

    // Wait for position fix (simple check: non-zero lat/lon)
    bool wait_for_position(int timeout_sec = 10) {
        for (int i = 0; i < timeout_sec * 5; ++i) {
            Telemetry::Position p = telemetry_->position();
            if (std::abs(p.latitude_deg) > 1e-7 || std::abs(p.longitude_deg) > 1e-7) {
                return true;
            }
            std::this_thread::sleep_for(200ms);
        }
        return false;
    }

    // ==================== MONITOR ====================
    void start_monitoring(double display_hz = 5.0) {
        monitoring_ = true;
        
        telemetry_->subscribe_position([this](Telemetry::Position pos){
            std::lock_guard<std::mutex> lock(status_mutex_);
            current_pos_ = pos;
        });
        telemetry_->subscribe_velocity_ned([this](Telemetry::VelocityNed vel){
            std::lock_guard<std::mutex> lock(status_mutex_);
            current_vel_ = vel;
        });
        telemetry_->subscribe_battery([this](Telemetry::Battery bat){
            std::lock_guard<std::mutex> lock(status_mutex_);
            current_battery_ = bat;
        });

        monitor_thread_ = std::thread([this, display_hz](){
            auto interval = std::chrono::milliseconds(int(1000.0 / display_hz));
            while (monitoring_) {
                auto now = std::chrono::system_clock::now();
                std::time_t now_c = std::chrono::system_clock::to_time_t(now);
                std::tm* timeinfo = std::localtime(&now_c);

                char time_str[32];
                std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", timeinfo);

                Telemetry::Position pos;
                Telemetry::VelocityNed vel;
                Telemetry::Battery bat;
                {
                    std::lock_guard<std::mutex> lock(status_mutex_);
                    pos = current_pos_;
                    vel = current_vel_;
                    bat = current_battery_;
                }

                float speed = std::sqrt(
                    vel.north_m_s*vel.north_m_s +
                    vel.east_m_s*vel.east_m_s +
                    vel.down_m_s*vel.down_m_s
                );

                std::string mname = get_mission_name();

                std::cout << "\r"
                    << "[" << time_str << "] "
                    << "Lat:" << std::setw(10) << std::fixed << std::setprecision(6) << pos.latitude_deg << "°"
                    << " Lon:" << std::setw(10) << std::fixed << std::setprecision(6) << pos.longitude_deg << "°"
                    << " Alt:" << std::setw(7)  << std::fixed << std::setprecision(2) << pos.relative_altitude_m << " m"
                    << " | Speed:" << std::setw(6) << std::fixed << std::setprecision(2) << speed << " m/s"
                    << " | Bat:"   << std::setw(5) << std::fixed << std::setprecision(1) << bat.remaining_percent << "%"
                    << " | Mission:" << std::setw(10) << std::left << mname
                    << std::right << std::flush;


                std::this_thread::sleep_for(interval);
            }
        });
    }

    void stop_monitoring() {
        monitoring_ = false;
        if (monitor_thread_.joinable()) monitor_thread_.join();
    }

    // ==================== ARM / TAKEOFF / LAND ====================
    bool arm() {
        auto res = action_->arm();
        if (res == Action::Result::Success) {
            std::cout << "\n Drone ARMED\n";
            return true;
        }
        std::cout << "\n ARM failed: " << "\n";
        return false;
    }

    bool disarm() {
        auto res = action_->disarm();
        if (res == Action::Result::Success) {
            std::cout << "\n Drone DISARMED\n";
            return true;
        }
        return false;
    }

    bool takeoff(float alt=10.0f) {
    if (!wait_for_position(10)) return false;
    
    if (action_->takeoff() != Action::Result::Success) return false;

    // Wait until drone is in air
    int waited_ms = 0;
    while (!telemetry_->in_air() && waited_ms < 10000) { // timeout 10s
        std::this_thread::sleep_for(200ms);
        waited_ms += 200;
    }

    if (!telemetry_->in_air()) {
        std::cout << "Takeoff failed: drone not in air\n";
        return false;
    }
    std::cout << "Drone TAKEOFF\n";
    return true;
}

    bool land() {
    if (action_->land() != Action::Result::Success) return false;

    int waited_ms = 0;
    while (telemetry_->landed_state() != Telemetry::LandedState::OnGround && waited_ms < 60000) {
        std::this_thread::sleep_for(200ms);
        waited_ms += 200;
    }

    if (telemetry_->landed_state() != Telemetry::LandedState::OnGround) {
        std::cout << "Land timeout!\n";
        return false;
    }

    std::cout << "Drone LANDED\n";
    return true;
}
    bool safe_land_and_disarm() {
        // Gửi lệnh hạ cánh
        if (action_->land() != Action::Result::Success) {
        std::cout << "\nLanding command failed!\n";
        return false;
        }

        // Chờ thật sự landed
        const int timeout_sec = 60;
        int waited = 0;
        while (telemetry_->landed_state() != Telemetry::LandedState::OnGround && waited < timeout_sec*1000) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            waited += 200;
        }

        if (telemetry_->landed_state() != Telemetry::LandedState::OnGround) {
            std::cout << "\nTimeout: drone did not land!\n";
            return false;
        }

        std::cout << "\nDrone LANDED\n";

        // Disarm
        if (action_->disarm() != Action::Result::Success) {
            std::cout << "Disarm failed!\n";
            return false;
        }

        std::cout << "Drone DISARMED\n";
        return true;

        }
    // ==================== MISSION ====================
    void stop_mission() {
        manual_mode_ = false; // ensure manual loop exits
        mission_running_ = false;
        if (mission_thread_.joinable()) mission_thread_.join();
        set_mission_name("None");
    }

    void start_circle_mission(float radius=10.0f, float alt=10.0f, float speed=1.0f) {
        if (mission_running_) return;
        mission_running_ = true;
        set_mission_name("Circle");

        mission_thread_ = std::thread([this, radius, alt, speed](){
            if (!wait_for_position(10)) {
                std::cout << "\nCircle mission aborted: no position fix\n";
                mission_running_ = false;
                set_mission_name("None");
                return;
            }

            Telemetry::Position start_pos = telemetry_->position();
            double lat0 = start_pos.latitude_deg;
            double lon0 = start_pos.longitude_deg;
            const double meters_to_lat = 1.0 / 111320.0;
            const double meters_to_lon = 1.0 / (111320.0 * std::cos(lat0 * M_PI / 180.0));

            double angle = 0.0;
            double dt = 1.0; // 1s interval to avoid spamming commands

            while (mission_running_) {
                double lat = lat0 + radius * std::cos(angle) * meters_to_lat;
                double lon = lon0 + radius * std::sin(angle) * meters_to_lon;
                auto res = action_->goto_location(lat, lon, alt, 0.0f);
                if (res != Action::Result::Success) {
                    std::cout << "\ngoto_location failed: " << "\n";
                }

                angle += (speed / std::max(radius, 0.001f)) * dt;
                if (angle > 2*M_PI) angle -= 2*M_PI;

                std::this_thread::sleep_for(std::chrono::milliseconds(int(dt*1000)));
            }

            set_mission_name("None");
        });
    }

    void start_square_mission(float edge=10.0f, float alt=10.0f, float speed=2.0f) {
        if (mission_running_) return;
        mission_running_ = true;
        set_mission_name("Square");

        mission_thread_ = std::thread([this, edge, alt, speed](){
            if (!wait_for_position(10)) {
                std::cout << "\nSquare mission aborted: no position fix\n";
                mission_running_ = false;
                set_mission_name("None");
                return;
            }

            Telemetry::Position start_pos = telemetry_->position();
            double lat0 = start_pos.latitude_deg;
            double lon0 = start_pos.longitude_deg;
            const double meters_to_lat = 1.0 / 111320.0;
            const double meters_to_lon = 1.0 / (111320.0 * std::cos(lat0 * M_PI / 180.0));

            std::vector<std::pair<double,double>> points = {
                {lat0 + edge*meters_to_lat, lon0},
                {lat0 + edge*meters_to_lat, lon0 + edge*meters_to_lon},
                {lat0, lon0 + edge*meters_to_lon},
                {lat0, lon0}
            };

            size_t i = 0;
            while (mission_running_) {
                double lat = points[i].first;
                double lon = points[i].second;
                auto res = action_->goto_location(lat, lon, alt, 0.0f);
                if (res != Action::Result::Success) {
                    std::cout << "\ngoto_location failed: " << "\n";
                }

                std::this_thread::sleep_for(std::chrono::seconds(5));
                i = (i+1)%points.size();
            }

            set_mission_name("None");
        });
    }

    void start_triangle_mission(float edge=10.0f, float alt=10.0f, float speed=2.0f) {
        if (mission_running_) return;
        mission_running_ = true;
        set_mission_name("Triangle");

        mission_thread_ = std::thread([this, edge, alt, speed](){
            if (!wait_for_position(10)) {
                std::cout << "\nTriangle mission aborted: no position fix\n";
                mission_running_ = false;
                set_mission_name("None");
                return;
            }

            Telemetry::Position start_pos = telemetry_->position();
            double lat0 = start_pos.latitude_deg;
            double lon0 = start_pos.longitude_deg;
            const double meters_to_lat = 1.0 / 111320.0;
            const double meters_to_lon = 1.0 / (111320.0 * std::cos(lat0 * M_PI / 180.0));

            double h = edge * std::sqrt(3)/2.0;

            std::vector<std::pair<double,double>> points = {
                {lat0, lon0},
                {lat0 + h*meters_to_lat, lon0 + edge*meters_to_lon/2},
                {lat0, lon0 + edge*meters_to_lon},
                {lat0, lon0}
            };

            size_t i = 0;
            while (mission_running_) {
                double lat = points[i].first;
                double lon = points[i].second;
                auto res = action_->goto_location(lat, lon, alt, 0.0f);
                if (res != Action::Result::Success) {
                    std::cout << "\ngoto_location failed: "  << "\n";
                }

                std::this_thread::sleep_for(std::chrono::seconds(5));
                i = (i+1)%points.size();
            }

            set_mission_name("None");
        });
    }

    void start_sine_mission(float amplitude=5.0f, float wavelength=10.0f, float alt=10.0f, float speed=1.0f) {
    if (mission_running_) return;
    mission_running_ = true;
    set_mission_name("Sine");

    mission_thread_ = std::thread([this, amplitude, wavelength, alt, speed](){
        if (!wait_for_position(10)) {
            std::cout << "\nSine mission aborted: no position fix\n";
            mission_running_ = false;
            set_mission_name("None");
            return;
        }

        Telemetry::Position start_pos = telemetry_->position();
        double lat0 = start_pos.latitude_deg;
        double lon0 = start_pos.longitude_deg;

        double x = 0.0;  // khoảng cách di chuyển theo kinh độ
        double dt = 1.0; // thời gian 1s giữa các lệnh

        while (mission_running_) {
            // chuyển đổi từ mét sang độ
            const double meters_to_lat = 1.0 / 111320.0;
            const double meters_to_lon = 1.0 / (111320.0 * std::cos(lat0 * M_PI / 180.0));

            // Drone chạy ngang theo kinh độ (trái → phải)
            double lon = lon0 + x * meters_to_lon;
            // Sóng sin lên xuống theo vĩ độ
            double lat = lat0 + amplitude * std::sin(2*M_PI*x / wavelength) * meters_to_lat;

            auto res = action_->goto_location(lat, lon, alt, 0.0f);
            if (res != Action::Result::Success) {
                std::cout << "\ngoto_location failed\n";
            }

            x += speed * dt; // tăng khoảng cách
            std::this_thread::sleep_for(std::chrono::milliseconds(int(dt*1000)));
        }

        set_mission_name("None");
    });
}

    // ==================== MANUAL CONTROL ====================
    void manual_control(float step_m=2.0f) {
        manual_mode_ = true;
        set_mission_name("Manual");
        if (!wait_for_position(10)) {
            std::cout << "\nManual control aborted: no position fix\n";
            manual_mode_ = false;
            set_mission_name("None");
            return;
        }

        Telemetry::Position start_pos = telemetry_->position();
        double lat = start_pos.latitude_deg;
        double lon = start_pos.longitude_deg;
        double alt = start_pos.relative_altitude_m;

        while (manual_mode_) {
            if (kbhit()) {
                char c = getch();
                const double meters_to_lat = 1.0 / 111320.0;
                const double meters_to_lon = 1.0 / (111320.0 * std::cos(lat * M_PI / 180.0));
                switch(c) {
                    case 'w': lat += step_m * meters_to_lat; break;
                    case 's': lat -= step_m * meters_to_lat; break;
                    case 'a': lon -= step_m * meters_to_lon; break;
                    case 'd': lon += step_m * meters_to_lon; break;
                    case 'r': alt += step_m; break;
                    case 'f': alt -= step_m; break;
                    case 'q': manual_mode_ = false; break;
                }
                auto res = action_->goto_location(lat, lon, alt, 0.0f);
                if (res != Action::Result::Success) {
                    std::cout << "\ngoto_location failed: " << "\n";
                }
            }
            std::this_thread::sleep_for(50ms);
        }

        set_mission_name("None");
    }
};

// ==================== MAIN ====================
int main() {
    Mavsdk::Configuration config(245, 0, true);
    Mavsdk mavsdk(config);
    ConnectionResult conn_result = mavsdk.add_any_connection("udp://:14540");
    if (conn_result != ConnectionResult::Success) {
        std::cerr << "Connection failed\n";
        return 1;
    }

    // wait for system
    while (mavsdk.systems().empty()) std::this_thread::sleep_for(1s);
    auto system = mavsdk.systems().at(0);

    DroneManager drone(system);
    drone.start_monitoring(5.0);

    setNonBlocking(true);
    bool running = true;

    std::cout << "\n===== DRONE CONTROL MENU =====\n"
              << "T: Takeoff & Arm\n"
              << "L: Land & Disarm\n"
              << "C: Circle mission\n"
              << "S: Square mission\n"
              << "1: Triangle mission\n"
              << "2: Sine mission\n"
              << "M: Manual control (WASD/RF)\n"
              << "X: Stop current mission\n"
              << "Q: Quit\n";

    while (running) {
        if (kbhit()) {
            char c = getch();
            switch (c) {
                case 't': case 'T': 
                    if (drone.arm()) drone.takeoff(10.0f); 
                    break;
                case 'l': case 'L': 
                    drone.safe_land_and_disarm(); 
                    break;
                case 'c': case 'C': 
                    drone.stop_mission(); 
                    drone.start_circle_mission(10.0f, 10.0f, 1.0f); 
                    break;
                case 's': case 'S': 
                    drone.stop_mission(); 
                    drone.start_square_mission(10.0f, 10.0f, 2.0f); 
                    break;
                case '1': 
                    drone.stop_mission(); 
                    drone.start_triangle_mission(10.0f, 10.0f, 2.0f); 
                    break;
                case '2': 
                    drone.stop_mission(); 
                    drone.start_sine_mission(5.0f, 10.0f, 10.0f, 1.0f); 
                    break;
                case 'm': case 'M': 
                    drone.stop_mission(); 
                    drone.manual_control(); 
                    break;
                case 'x': case 'X': 
                    drone.stop_mission(); 
                    break;
                case 'q': case 'Q': 
                    running = false; 
                    break;
            }
        }
        std::this_thread::sleep_for(50ms);
    }

    drone.stop_mission();
    drone.safe_land_and_disarm(); 
    drone.stop_monitoring();
    setNonBlocking(false);

    std::cout << "\nExiting...\n";
    return 0;
}