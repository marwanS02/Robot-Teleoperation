// .\build\Release\udp_tx_rx_sines_imgui.exe                          
#define _USE_MATH_DEFINES
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <string>
#include <algorithm>  // for std::clamp
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <limits>


       
#if defined(_WIN32)
  #include <winsock2.h>
  #include <ws2tcpip.h>
  using socklen_t = int;
  using socket_t  = SOCKET;
  #pragma comment(lib, "ws2_32.lib")
  #define WIN32_LEAN_AND_MEAN
  #include <windows.h>
  #include <tchar.h>
  #include <d3d11.h>
  #include <dxgi.h>
  #pragma comment(lib, "d3d11.lib")
  #pragma comment(lib, "d3dcompiler.lib")
  #pragma comment(lib, "dxgi.lib")
#else
  #error "This sample targets Windows (Win32 + DirectX11)."
#endif
// ---- kill stray Windows-style min/max macros from any header ----
#ifdef min
  #undef min
#endif
#ifdef max
  #undef max
#endif

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif
constexpr double TAU = 6.28318530717958647692;
static constexpr double W_FIXED_YZ  = 0.1;   // rad/s (100% orientation speed)
static constexpr double V_MAX_CART  = 0.03;  // m/s (100% Cartesian velocity)

// ---------------- CONFIG ----------------
static const char*  LINUX_IP        = "100.108.230.113";
//static const char*  LINUX_IP        = "192.168.4.100";
static const uint16_t SEND_PORT     = 5005;
static const uint16_t RECV_PORT     = 5006;
static const double FREQ_HZ         = 1.0;
// static const int    IO_RATE_HZ      = 100;
// static const size_t MAX_POINTS      = 600;   // ~6s at 100 Hz
static const int    IO_RATE_HZ      = 300;
static const size_t MAX_POINTS      = 1800;  // ~6s at 300 Hz

static const bool   HOLD_ON_TIMEOUT = true;
static const bool   ENABLE_LOCAL_ECHO_TEST = false;
// ------- BRIDGES: Python class + Teensy serial -------
static const char*  CLASS_UDP_IP    = "127.0.0.1";
static const uint16_t CLASS_UDP_PORT = 6001;
static const uint16_t POLL_PORT = 5007;   // robot's poll port
static const uint16_t HSTATE_UDP_PORT = 6010;  // Python will listen here
// Python magnet-tracker UDP pose input
static const uint16_t PY_POSE_UDP_PORT = 6011;  // must match POSE_UDP_PORT in Python
static double         g_last_logged_time = -1.0;  // NEW: last time written to CSV

// --- Classifier watchdog ---
static const double CLASS_RX_TIMEOUT_S = 0.75;  // 750 ms of silence => REST
static std::atomic<double> g_last_class_rx_s{-1.0};
static double now_steady_s() {
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()
  ).count();
}
static std::atomic<double> g_last_rtt_ms{NAN};  // round-trip time in ms
// --- RTT statistics over a short window ---
static std::mutex g_rtt_mutex;
static std::deque<std::pair<double,double>> g_rtt_samples; // (time_s, rtt_ms)
static double g_rtt_avg_ms         = NAN;
static double g_rtt_std_ms         = NAN;
static double g_rtt_last_compute_s = 0.0;  // last time we recomputed stats

// --- Pose (vx,vy,vz) watchdog ---
static const double POSE_RX_TIMEOUT_S = 0.5;   // e.g. 500 ms of silence => no motion
static std::atomic<double> g_last_pose_rx_s{-1.0};

// --- Robot RX watchdog (poll reply / telemetry) ---
static const double ROBOT_RX_TIMEOUT_S = 0.5;  // e.g. 500 ms without RX => "disconnected"
static std::atomic<double> g_last_robot_rx_s{-1.0};

// --- Robot pose from UDP telemetry (indices 10..15 of 16D packet) ---
static std::atomic<double> g_robot_x{NAN};      // pkt[10]
static std::atomic<double> g_robot_z{NAN};      // pkt[11]
static std::atomic<double> g_robot_y{NAN};      // pkt[12]
static std::atomic<double> g_robot_roll{NAN};   // pkt[13] [rad]
static std::atomic<double> g_robot_pitch{NAN};  // pkt[14] [rad]
static std::atomic<double> g_robot_yaw{NAN};    // pkt[15] [rad]



// --- Serial reconnect config & status ---
static const DWORD  ARDUINO_BAUD  = 115200;
static const char*  ARDUINO_COM   = R"(\\.\COM3)";   // keep trying this name
static const int    ARDUINO_RETRY_MS = 1000;         // wait between open attempts
static const double ARDUINO_SEND_HZ  = 300.0;        // TX rate to Arduino
static std::atomic<bool> g_serial_connected{false};  // for UI/status

static constexpr double BEND_MIN_RAW = 220.0;  // raw bend sensor min (calibrated)
static constexpr double BEND_MAX_RAW = 350.0;  // raw bend sensor max (calibrated)
static constexpr double GRIPPER_MIN  = 0.000;   // gripper command min
static constexpr double GRIPPER_MAX  = 0.080;   // gripper command max



// Arduino line format (tolerant parser expects: "v1,v2\n" or "anything,v1,v2")

// ---------------------------------------

static int set_nonblocking(socket_t s) {
  u_long mode = 1;
  return ioctlsocket(s, FIONBIO, &mode);
}

static void closesock(socket_t s) { closesocket(s); }


// Ring buffer
template<typename T>
struct Ring {
  explicit Ring(size_t cap): cap_(cap) {}
  void push(const T& x) { if (dq_.size()==cap_) dq_.pop_front(); dq_.push_back(x); }
  std::vector<T> snapshot() const { return {dq_.begin(), dq_.end()}; }
  bool empty() const { return dq_.empty(); }
  const T& back() const { return dq_.back(); }
  size_t size() const { return dq_.size(); }
private:
  size_t cap_; std::deque<T> dq_;
};

struct SharedBuffers {
  Ring<double> t_hist{MAX_POINTS};
  std::array<Ring<double>,3> tx_hist{
    Ring<double>(MAX_POINTS), Ring<double>(MAX_POINTS), Ring<double>(MAX_POINTS)
  };
  // Velocity history (vx, vy, vz)
  std::array<Ring<double>,3> vel_hist{
    Ring<double>(MAX_POINTS), Ring<double>(MAX_POINTS), Ring<double>(MAX_POINTS)
  };
  // ==== expanded to 12 RX channels ====
// 0: tau_toolx
// 1: feedback
// 2: hstate
// 3: wx_tool_deg
// 4: grip_width
// 5: wy_prop
// 6: wz_prop
// 7: vx_tool_meas
// 8: vy_tool_meas
// 9: vz_tool_meas
// 10: F_axial along tool-X [N]
// 11: F_radial in tool Y–Z plane [N]
  std::array<Ring<double>,12> rx_hist{
    Ring<double>(MAX_POINTS), Ring<double>(MAX_POINTS), Ring<double>(MAX_POINTS),
    Ring<double>(MAX_POINTS), Ring<double>(MAX_POINTS), Ring<double>(MAX_POINTS),
    Ring<double>(MAX_POINTS), Ring<double>(MAX_POINTS), Ring<double>(MAX_POINTS),
    Ring<double>(MAX_POINTS), Ring<double>(MAX_POINTS), Ring<double>(MAX_POINTS)
  };
  std::array<Ring<double>,2> f_hist{
    Ring<double>(MAX_POINTS), Ring<double>(MAX_POINTS)
  };
  std::mutex mtx;
  std::array<double,12> last_rx{
    NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN
  };
};



static std::atomic<bool> running{true};

// Values coming from Arduino (PC reads them)
static std::atomic<double> g_arduino_v1{0.0};
static std::atomic<double> g_arduino_v2{0.0};
static std::atomic<double> g_class_idx{0.0}; // will be 0..4 as double
static std::atomic<double> g_arduino_f1{0.0};  // NEW
static std::atomic<double> g_arduino_f2{0.0};  // NEW
// Commands going back to Arduino (PC writes them)
static std::atomic<double> g_cmd1{0.0}; // RX1 to Arduino
static std::atomic<double> g_cmd2{0.0}; // RX2 to Arduino
// ===== Teensy pose on COM11: (x,y,z,theta_x,theta_y,theta_z) =====
static const char* TEENSY_COM  = R"(\\.\COM11)";
static const DWORD TEENSY_BAUD = 115200;
static std::atomic<bool> g_teensy_connected{false};

static std::atomic<double> g_x{0.0};
static std::atomic<double> g_y{0.0};
static std::atomic<double> g_z{0.0};
static std::atomic<double> g_thx{0.0};
static std::atomic<double> g_thy{0.0};
static std::atomic<double> g_thz{0.0};

// Pack/unpack <3d little-endian (24 bytes)
static void pack3d_le(const std::array<double,3>& v, uint8_t out[24]) {
  static_assert(sizeof(double)==8,"Expect 64-bit double");
  std::memcpy(out+0,&v[0],8); std::memcpy(out+8,&v[1],8); std::memcpy(out+16,&v[2],8);
}

static void pack6d_le(const std::array<double,6>& v, uint8_t out[48]) {
  static_assert(sizeof(double) == 8, "Expect 64-bit double");
  for (int i = 0; i < 6; ++i) {
    std::memcpy(out + 8*i, &v[i], 8);
  }
}


static void unpack3d_le(const uint8_t in[24], std::array<double,3>& v) {
  std::memcpy(&v[0],in+0,8); std::memcpy(&v[1],in+8,8); std::memcpy(&v[2],in+16,8);
}
// ==== optional 6D RX (48 bytes) ====
static void unpack6d_le(const uint8_t in[48], std::array<double,6>& v) {
  for (int i=0;i<6;++i) std::memcpy(&v[i], in+8*i, 8);
}


// ==== 7D RX (56 bytes) ====
static void unpack7d_le(const uint8_t* in, std::array<double,7>& v) {
  for (int i=0;i<7;++i) std::memcpy(&v[i], in+8*i, 8);
}

// ==== 10D RX (80 bytes) ====
static void unpack10d_le(const uint8_t* in, std::array<double,10>& v) {
  for (int i=0;i<10;++i) {
    std::memcpy(&v[i], in + 8*i, 8);
  }
}

// ==== 16D RX (128 bytes) ====
static void unpack16d_le(const uint8_t* in, std::array<double,16>& v) {
  for (int i=0; i<16; ++i) {
    std::memcpy(&v[i], in + 8*i, 8);
  }
}

// ==== 18D RX (144 bytes) ====
static void unpack18d_le(const uint8_t* in, std::array<double,18>& v) {
  for (int i = 0; i < 18; ++i) {
    std::memcpy(&v[i], in + 8*i, 8);
  }
}


struct VelocityCmd {
  double vx;
  double vy;
  double vz;
  bool   flag;
};

static VelocityCmd compute_velocity(double x, double y, double z,
                                    double theta_x, double theta_y, double theta_z)
{
    VelocityCmd out{};

    // --------------------------------------------------------------------
    // 1) Orientation gating: use (theta_x, theta_y, theta_z) as (mx, my, mz)
    //    Same logic as Python: pretty_angle = arccos(|mz|) in degrees.
    // --------------------------------------------------------------------
    double mx = theta_x;
    double my = theta_y;
    double mz = theta_z;

    // Normalize orientation vector to be safe
    double m_norm = std::sqrt(mx*mx + my*my + mz*mz);
    if (m_norm > 1e-6) {
        mx /= m_norm;
        my /= m_norm;
        mz /= m_norm;
    } else {
        // Degenerate orientation → no activation, zero velocity
        out.vx   = 0.0;
        out.vy   = 0.0;
        out.vz   = 0.0;
        out.flag = false;
        return out;
    }

    double cos_a = std::fabs(mz);                 // |mz|
    cos_a = std::clamp(cos_a, -1.0, 1.0);
    double angle_rad = std::acos(cos_a);          // radians
    double angle_deg = angle_rad * 180.0 / M_PI;  // 0 .. 90

    // Threshold at 60 degrees: above → active, below → inactive
    bool active = (angle_deg > 60.0);
    out.flag = active;

    // --------------------------------------------------------------------
    // 2) Position → normalized velocities in [-1, 1]
    //    Clamp x,y,z to [-0.1, 0.1] m, then map:
    //    norm_x = x/0.1, norm_y = y/0.1, norm_z = z/0.1
    //    vx = -norm_y, vy = -norm_x, vz = norm_z
    // --------------------------------------------------------------------
    const double POS_MAX = 0.1;    // meters
    const double POS_MIN = -0.1;

    double x_clamp = std::clamp(x, POS_MIN, POS_MAX);
    double y_clamp = std::clamp(y, POS_MIN, POS_MAX);
    double z_clamp = std::clamp(z, POS_MIN, POS_MAX);

    double x_prop = (std::fabs(POS_MAX) > 1e-9) ? (x_clamp / POS_MAX) : 0.0;  // [-1,1]
    double y_prop = (std::fabs(POS_MAX) > 1e-9) ? (y_clamp / POS_MAX) : 0.0;  // [-1,1]
    double z_prop = (std::fabs(POS_MAX) > 1e-9) ? (z_clamp / POS_MAX) : 0.0;  // [-1,1]

    // Your mapping / sign conventions:
    //   vx mapped to y, vy mapped to x, with:
    //   x_max → -vy_max, -x_max → +vy_max
    //   y_max → -vx_max, -y_max → +vx_max
    double vx_prop = -x_prop;
    double vy_prop = y_prop;
    double vz_prop =  -z_prop;   // robot Z inversion happens later in io_thread_fn

    // Clamp just in case of numerical edge cases
    out.vx = std::clamp(vx_prop, -1.0, 1.0);
    out.vy = std::clamp(vy_prop, -1.0, 1.0);
    out.vz = std::clamp(vz_prop, -1.0, 1.0);

    // NOTE: we do NOT zero them here when inactive.
    // io_thread_fn already does:
    //   vel.flag ? vel.vx : 0.0, etc.
    return out;
}




// ---------------- Networking thread ----------------
void io_thread_fn(SharedBuffers* buf) {
  // Enforce 1 s of REST (class 0) between different non-zero classes
  static int    s_active_cls    = 0;      // last accepted (non-rest) class
  static bool   s_in_rest_gate  = false;  // currently forcing REST between classes
  static double s_rest_start_s  = 0.0;    // steady-clock time when gate started
  constexpr double REST_GATE_HOLD_S = 1.0; // seconds

  WSADATA wsa;
  if (WSAStartup(MAKEWORD(2,2), &wsa)!=0) { std::cerr<<"WSAStartup failed\n"; return; }

  socket_t sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd == INVALID_SOCKET) { std::cerr<<"socket() failed\n"; WSACleanup(); return; }
  // Poll socket/address (can reuse sockfd; we'll just use same socket to send)
  sockaddr_in poll_addr{}; poll_addr.sin_family = AF_INET;
  inet_pton(AF_INET, LINUX_IP, &poll_addr.sin_addr);
  poll_addr.sin_port = htons(POLL_PORT);

  sockaddr_in recv_addr{}; recv_addr.sin_family=AF_INET;
  recv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  recv_addr.sin_port = htons(RECV_PORT);
  if (bind(sockfd,(sockaddr*)&recv_addr,sizeof(recv_addr))<0) {
    std::cerr<<"bind(RECV_PORT) failed\n"; closesock(sockfd); WSACleanup(); return;
  }
  set_nonblocking(sockfd);

  sockaddr_in send_addr{}; send_addr.sin_family=AF_INET;
  inet_pton(AF_INET, LINUX_IP, &send_addr.sin_addr);
  send_addr.sin_port = htons(SEND_PORT);

  // --- New: destination for hstate (RX3) → Python bridge ---
  sockaddr_in hstate_addr{};
  hstate_addr.sin_family = AF_INET;
  inet_pton(AF_INET, "127.0.0.1", &hstate_addr.sin_addr);  // local Python
  hstate_addr.sin_port = htons(HSTATE_UDP_PORT);


  const double dt = 1.0 / double(IO_RATE_HZ);
  auto t0 = std::chrono::steady_clock::now();
  auto next_tick = t0;

  while (running.load(std::memory_order_relaxed)) {
    auto now = std::chrono::steady_clock::now();
    if (now < next_tick) std::this_thread::sleep_for(next_tick - now);
    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dt));

    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    // New (two from Arduino, third from Python classifier):
    // Decide safe class based on last receive time
    double cls = g_class_idx.load(std::memory_order_relaxed);
    double last = g_last_class_rx_s.load(std::memory_order_relaxed);
    double tnow = now_steady_s();
    if (last < 0.0 || (tnow - last) > CLASS_RX_TIMEOUT_S) {
      cls = 0.0; // REST while classifier is disconnected/silent
    }
    double raw_tx1 = g_arduino_v1.load(std::memory_order_relaxed)-0.85;
    // linear map: [-70,70] -> [-110,110]
    constexpr double k = 110.0 / 70.0;   
    double tx1_mapped = std::clamp(raw_tx1 * k, -110.0, 110.0);

    double tx2_val   = g_arduino_v2.load(std::memory_order_relaxed);
    double tx2_mapped = (tx2_val - BEND_MIN_RAW) / (BEND_MAX_RAW - BEND_MIN_RAW);
    tx2_mapped = std::clamp(tx2_mapped, 0.0, 1.0);
    tx2_mapped = GRIPPER_MIN + tx2_mapped * (GRIPPER_MAX - GRIPPER_MIN);

    // ===== Teensy/Python pose → velocity command with watchdog =====
    double x    = g_x.load(std::memory_order_relaxed);
    double y    = g_y.load(std::memory_order_relaxed);
    double z    = g_z.load(std::memory_order_relaxed);
    double th_x = g_thx.load(std::memory_order_relaxed);
    double th_y = g_thy.load(std::memory_order_relaxed);
    double th_z = g_thz.load(std::memory_order_relaxed);

    // Treat (x,y,z,th_x,th_y,th_z) as (vx,vy,vz,mx,my,mz) for gating.
    auto is_zero = [](double v) { return std::fabs(v) < 1e-8; };

    bool all_pose_zero =
        is_zero(x)    &&
        is_zero(y)    &&
        is_zero(z)    &&
        is_zero(th_x) &&
        is_zero(th_y) &&
        is_zero(th_z);

    // --- NEW: gate classifier by magnet state ---
    // If ANY of (vx,vy,vz,mx,my,mz) ≠ 0  → force REST (0.0)
    // Only when ALL six == 0.0          → allow real cls
    if (!all_pose_zero) {
        cls = 0.0;   // REST while magnet is "active"
    }
    
        // --- Class transition rest-gate: enforce 1 s of REST between different active classes ---
    {
        // cls already includes timeout and magnet gating
        int raw_cls = static_cast<int>(std::lround(cls));
        if (raw_cls < 0) raw_cls = 0;
        if (raw_cls > 4) raw_cls = 4;

        double tnow_s = now_steady_s();

        if (!s_in_rest_gate) {
            // Normal mode: decide whether to start a rest gate
            // Trigger ONLY when going from one non-zero class to another non-zero class.
            if (s_active_cls != 0 && raw_cls != 0 && raw_cls != s_active_cls) {
                // Start enforced REST
                s_in_rest_gate  = true;
                s_rest_start_s  = tnow_s;
                cls             = 0.0;   // force REST now
            } else {
                // Pass class through
                cls          = static_cast<double>(raw_cls);
                s_active_cls = raw_cls;  // track last accepted class (can be 0)
            }
        } else {
            // We are currently enforcing a REST period
            double dt_gate = tnow_s - s_rest_start_s;
            if (dt_gate >= REST_GATE_HOLD_S) {
                // Rest period over: let whatever is present *now* pass
                s_in_rest_gate = false;
                cls            = static_cast<double>(raw_cls);
                s_active_cls   = raw_cls;   // may become 0 or a new class
            } else {
                // Still within enforced rest window
                cls = 0.0;
            }
        }
    }

    // Check pose freshness
    double pose_last = g_last_pose_rx_s.load(std::memory_order_relaxed);
    double pose_age  = (pose_last < 0.0) ? 1e9 : (now_steady_s() - pose_last);
    bool pose_ok     = (pose_age <= POSE_RX_TIMEOUT_S);

    VelocityCmd vel = compute_velocity(x, y, z, th_x, th_y, th_z);

    // If pose is stale → disable motion
    if (!pose_ok) {
        vel.flag = false;
    }


    // Values that go into the UDP packet (gated)
    std::array<double,3> vel_sent{
      vel.flag ? vel.vx : 0.0, 
      vel.flag ? vel.vy : 0.0,
      vel.flag ? vel.vz : 0.0
    };
    vel_sent[2] = -vel_sent[2]; // invert Z for robot frame

    // First 3 entries remain as before (J7, gripper, class),
    // last 3 entries are (vx,vy,vz) from the Teensy
    // overwrite whatever was computed, just for test

    
    std::array<double,6> tx_vals{
      tx1_mapped, tx2_mapped, cls,
      vel_sent[0], vel_sent[1], vel_sent[2]
    };


    // std::array<double,3> tx_vals{
    //   g_arduino_v1.load(std::memory_order_relaxed),
    //   g_arduino_v2.load(std::memory_order_relaxed),
    //   cls
    // };


    uint8_t pkt[48];
    pack6d_le(tx_vals, pkt);
    (void)::sendto(sockfd, (const char*)pkt, 48, 0, (sockaddr*)&send_addr, sizeof(send_addr));

    // --- trigger robot poll reply (required for telemetry + RTT) ---
    // Pack current steady-clock time into first double; robot should echo it
    // back in a 24-byte reply on RECV_PORT so we can measure round-trip time.
    std::array<double,3> ping_vals{
        now_steady_s(),  // timestamp in seconds (steady clock)
        0.0,
        0.0
    };
    uint8_t ping[24];
    pack3d_le(ping_vals, ping);

    (void)::sendto(sockfd, (const char*)ping, sizeof(ping), 0,
                   (sockaddr*)&poll_addr, sizeof(poll_addr));


    // Drain RX (keep latest). Support 24B (3d), 48B (6d), or 56B (7d).
    std::array<double,12> latest{};
    bool got_any = false;
    {
      std::scoped_lock lk(buf->mtx);
      // start from last → if we get partial (3d or 6d or 7d), we keep previous others
      latest = buf->last_rx;
    }


    for (;;) {
      fd_set rfds; FD_ZERO(&rfds); FD_SET(sockfd,&rfds);
      timeval tv{}; tv.tv_sec=0; tv.tv_usec=10000;
      int r = select(0,&rfds,nullptr,nullptr,&tv);
      if (r<=0) break;

      uint8_t rxbuf[1024];
      sockaddr_in from{}; socklen_t fromlen=sizeof(from);
      int n = recvfrom(sockfd,(char*)rxbuf,sizeof(rxbuf),0,(sockaddr*)&from,&fromlen);
      if (n == 24) {
          std::array<double,3> t3{};
          unpack3d_le(rxbuf, t3);

          // Treat t3[0] as echoed timestamp from poll reply → RTT
          double t_echo  = t3[0];
          double now_rtt = now_steady_s();
          if (t_echo > 0.0 && t_echo < now_rtt + 10.0) {  // basic sanity check
              double rtt_ms = (now_rtt - t_echo) * 1000.0;
              g_last_rtt_ms.store(rtt_ms, std::memory_order_relaxed);

              // Store sample for 3 s window stats
              {
                  std::lock_guard<std::mutex> lk(g_rtt_mutex);
                  g_rtt_samples.emplace_back(now_rtt, rtt_ms);
              }
          }

          // Mark robot as alive for watchdog if you want:
          got_any = true;

          // ✅ DO NOT write t3[] into latest[0..2] here.
          // Just continue to the next recv.
          continue;
           } else if (n == 48) {
        std::array<double,6> t6{}; 
        unpack6d_le(rxbuf, t6);
        for (int i=0; i<6; ++i) latest[i] = t6[i];
        got_any = true;

      } else if (n == 56) {
        std::array<double,7> t7{};
        unpack7d_le(rxbuf, t7);
        for (int i=0; i<7; ++i) latest[i] = t7[i];
        got_any = true;

      } else if (n == 144) {  // 18 doubles: full telemetry with pose + forces
        std::array<double,18> t18{};
        unpack18d_le(rxbuf, t18);

        // First 10 channels into the usual telemetry slots (0..9)
        for (int i = 0; i < 10; ++i) {
          latest[i] = t18[i];
        }

        // Pose (exact order from UDP: x, z, y, roll, pitch, yaw) at indices 10..15
        g_robot_x.store(t18[10], std::memory_order_relaxed);
        g_robot_z.store(t18[11], std::memory_order_relaxed);
        g_robot_y.store(t18[12], std::memory_order_relaxed);
        g_robot_roll.store(t18[13], std::memory_order_relaxed);
        g_robot_pitch.store(t18[14], std::memory_order_relaxed);
        g_robot_yaw.store(t18[15], std::memory_order_relaxed);

        // New forces: axial and radial at indices 16,17
        latest[10] = t18[16];   // F_axial along tool-X [N]
        latest[11] = t18[17];   // F_radial in tool Y–Z [N]

        got_any = true;

      } else if (n == 128) {  // 16 doubles: full telemetry with pose (no forces)
        std::array<double,16> t16{};
        unpack16d_le(rxbuf, t16);

        // First 10 channels into the usual telemetry slots
        for (int i = 0; i < 10; ++i) {
          latest[i] = t16[i];
        }

        // Pose (exact order from UDP: x, z, y, roll, pitch, yaw)
        g_robot_x.store(t16[10], std::memory_order_relaxed);
        g_robot_z.store(t16[11], std::memory_order_relaxed);
        g_robot_y.store(t16[12], std::memory_order_relaxed);
        g_robot_roll.store(t16[13], std::memory_order_relaxed);
        g_robot_pitch.store(t16[14], std::memory_order_relaxed);
        g_robot_yaw.store(t16[15], std::memory_order_relaxed);

        // No force channels in 16D packet → mark as NaN
        latest[10] = NAN;
        latest[11] = NAN;

        got_any = true;

      } else if (n == 80) {   // legacy: 10 doubles only
        std::array<double,10> t10{};
        unpack10d_le(rxbuf, t10);

        // Copy the 10 telemetry values into the 12-slot buffer
        for (int i = 0; i < 10; ++i) {
          latest[i] = t10[i];
        }

        // No pose, no forces in this legacy packet
        latest[10] = NAN;
        latest[11] = NAN;

        got_any = true;
      }


    }

    // Robot RX watchdog and safe fall-back
    double tnow_robot = now_steady_s();
    bool robot_ok = true;

    if (got_any) {
      // We received at least one telemetry frame this cycle -> robot alive
      g_last_robot_rx_s.store(tnow_robot, std::memory_order_relaxed);
    } else {
      // No frames this cycle -> check how long since last robot RX
      double last_robot = g_last_robot_rx_s.load(std::memory_order_relaxed);
      double age = (last_robot < 0.0) ? 1e9 : (tnow_robot - last_robot);
      robot_ok = (age <= ROBOT_RX_TIMEOUT_S);
    }

    if (!robot_ok) {
      // Robot considered disconnected:
      //  - force hstate = 1 (DISCONNECTED)
      //  - force both channels back to Arduino to 0
      latest[0] = 0.0;   // RX1  → torque / whatever you send to Arduino
      latest[1] = 0.0;   // RX2  → position / grip feedback etc.
      latest[2] = 1.0;   // RX3  → hstate = 1 (error / disconnect)
    } else if (!got_any && !HOLD_ON_TIMEOUT) {
      // Optional: if you don't want hold-on-timeout when still "ok"
      for (double &v: latest) v = NAN;
    }


    // --- New: send RX3 (hstate) as float32 to Python via UDP ---
    {
      float hstate_f = static_cast<float>(latest[2]);  // RX2 index = 2 (hstate)
      (void)::sendto(
          sockfd,
          reinterpret_cast<const char*>(&hstate_f),
          sizeof(hstate_f),
          0,
          reinterpret_cast<sockaddr*>(&hstate_addr),
          sizeof(hstate_addr)
      );
    }

    // Relay a couple of channels back to Arduino (unchanged)
    g_cmd1.store(latest[0], std::memory_order_relaxed); // RX1
    g_cmd2.store(latest[1], std::memory_order_relaxed); // RX2


    { 
      std::scoped_lock lk(buf->mtx);
      buf->t_hist.push(t);

      // First 3 TX channels as before
      for (int i=0; i<3; ++i) buf->tx_hist[i].push(tx_vals[i]);

      // Velocity history: store *raw* vx,vy,vz from compute_velocity (not gated)
      buf->vel_hist[0].push(vel_sent[0]);
      buf->vel_hist[1].push(vel_sent[1]);
      buf->vel_hist[2].push(vel_sent[2]);


      for (int i=0; i<12; ++i)
        buf->rx_hist[i].push(latest[i]);  // now 12!



      // NEW: log F1/F2 (can be NAN if legacy frame)
      buf->f_hist[0].push(g_arduino_f1.load(std::memory_order_relaxed));
      buf->f_hist[1].push(g_arduino_f2.load(std::memory_order_relaxed));

      buf->last_rx = latest;
    }

  }

  closesock(sockfd);
  WSACleanup();
}


// Optional local echo (listen SEND_PORT; echo to 127.0.0.1:RECV_PORT)
void local_echo_thread_fn() {
  WSADATA wsa; if (WSAStartup(MAKEWORD(2,2), &wsa)!=0) { std::cerr<<"[echo] WSAStartup failed\n"; return; }
  socket_t s = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (s == INVALID_SOCKET) { std::cerr<<"[echo] socket() failed\n"; WSACleanup(); return; }

  sockaddr_in bind_addr{}; bind_addr.sin_family=AF_INET;
  bind_addr.sin_addr.s_addr=htonl(INADDR_ANY);
  bind_addr.sin_port=htons(SEND_PORT);
  if (bind(s,(sockaddr*)&bind_addr,sizeof(bind_addr))<0) {
    std::cerr<<"[echo] bind failed\n"; closesock(s); WSACleanup(); return;
  }
  set_nonblocking(s);

  sockaddr_in out{}; out.sin_family=AF_INET;
  inet_pton(AF_INET,"127.0.0.1",&out.sin_addr);
  out.sin_port=htons(RECV_PORT);
  std::cout << "[echo] Local echo: " << SEND_PORT << " -> 127.0.0.1:" << RECV_PORT << "\n";

  while (running.load(std::memory_order_relaxed)) {
    fd_set rfds; FD_ZERO(&rfds); FD_SET(s,&rfds);
    timeval tv{}; tv.tv_sec=0; tv.tv_usec=20000;
    int r = select(0,&rfds,nullptr,nullptr,&tv);
    if (r>0 && FD_ISSET(s,&rfds)) {
      uint8_t buf[1024];
      sockaddr_in from{}; socklen_t fromlen=sizeof(from);
      int n = recvfrom(s,(char*)buf,sizeof(buf),0,(sockaddr*)&from,&fromlen);
      if (n>0) sendto(s,(const char*)buf,n,0,(sockaddr*)&out,sizeof(out));
    }
  }
  closesock(s); WSACleanup();
}

// ------------------- ImGui + ImPlot (DX11) -------------------
#include "imgui.h"
#include "implot.h"
#include "backends/imgui_impl_win32.h"
#include "backends/imgui_impl_dx11.h"

static ID3D11Device*           g_pd3dDevice = nullptr;
static ID3D11DeviceContext*    g_pd3dDeviceContext = nullptr;
static IDXGISwapChain*         g_pSwapChain = nullptr;
static ID3D11RenderTargetView* g_mainRenderTargetView = nullptr;

extern LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
static LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
  if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
    return true;
  switch (msg) {
    case WM_SIZE:
      if (g_pd3dDevice != NULL && wParam != SIZE_MINIMIZED) {
        if (g_mainRenderTargetView) { g_mainRenderTargetView->Release(); g_mainRenderTargetView = NULL; }
        g_pSwapChain->ResizeBuffers(0, (UINT)LOWORD(lParam), (UINT)HIWORD(lParam), DXGI_FORMAT_UNKNOWN, 0);
        ID3D11Texture2D* pBackBuffer;
        g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer));
        g_pd3dDevice->CreateRenderTargetView(pBackBuffer, NULL, &g_mainRenderTargetView);
        pBackBuffer->Release();
      }
      return 0;
    case WM_DESTROY: PostQuitMessage(0); return 0;
    default: return DefWindowProc(hWnd, msg, wParam, lParam);
  }
}

// Helpers (top of file, or near plotting code)
static void ema(const std::vector<double>& x, double alpha, std::vector<double>& y) {
    y.resize(x.size());
    if (x.empty()) return;
    double s = x[0];
    for (size_t i = 0; i < x.size(); ++i) { s = alpha * x[i] + (1.0 - alpha) * s; y[i] = s; }
}
// alpha from cutoff f_c (Hz) and dt (s): alpha = 1 - exp(-2*pi*f_c*dt)
static double ema_alpha(double fc, double dt) {
    if (fc <= 0.0 || dt <= 0.0) return 1.0; // no smoothing if invalid
    return 1.0 - std::exp(-2.0 * M_PI * fc * dt);
}


static bool CreateDeviceD3D(HWND hWnd) {
  DXGI_SWAP_CHAIN_DESC sd{};
  sd.BufferCount = 2;
  sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
  sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
  sd.OutputWindow = hWnd;
  sd.SampleDesc.Count = 1;
  sd.Windowed = TRUE;
  sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

  UINT createDeviceFlags = 0;
  D3D_FEATURE_LEVEL featureLevel;
  const D3D_FEATURE_LEVEL featureLevelArray[2] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0, };
  if (D3D11CreateDeviceAndSwapChain(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, createDeviceFlags,
      featureLevelArray, 2, D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext) != S_OK)
    return false;

  ID3D11Texture2D* pBackBuffer;
  g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer));
  g_pd3dDevice->CreateRenderTargetView(pBackBuffer, NULL, &g_mainRenderTargetView);
  pBackBuffer->Release();
  return true;
}
static void CleanupDeviceD3D() {
  if (g_mainRenderTargetView) { g_mainRenderTargetView->Release(); g_mainRenderTargetView = NULL; }
  if (g_pSwapChain)           { g_pSwapChain->Release(); g_pSwapChain = NULL; }
  if (g_pd3dDeviceContext)    { g_pd3dDeviceContext->Release(); g_pd3dDeviceContext = NULL; }
  if (g_pd3dDevice)           { g_pd3dDevice->Release(); g_pd3dDevice = NULL; }
}


void class_rx_thread_fn() {
  WSADATA wsa;
  if (WSAStartup(MAKEWORD(2,2), &wsa)!=0) { std::cerr<<"[class] WSAStartup failed\n"; return; }

  socket_t s = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (s == INVALID_SOCKET) { std::cerr<<"[class] socket failed\n"; WSACleanup(); return; }

  sockaddr_in addr{}; addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(CLASS_UDP_PORT);
  if (bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) {
    std::cerr<<"[class] bind failed\n"; closesock(s); WSACleanup(); return;
  }
  set_nonblocking(s);
  std::cout << "[class] Listening on udp://" << CLASS_UDP_IP << ":" << CLASS_UDP_PORT << " for class index\n";

  char buf[128];
  while (running.load(std::memory_order_relaxed)) {
    fd_set rfds; FD_ZERO(&rfds); FD_SET(s,&rfds);
    timeval tv{}; tv.tv_sec=0; tv.tv_usec=20000;
    int r = select(0, &rfds, nullptr, nullptr, &tv);
    if (r > 0 && FD_ISSET(s, &rfds)) {
      sockaddr_in from{}; socklen_t flen = sizeof(from);
      int n = recvfrom(s, buf, sizeof(buf)-1, 0, (sockaddr*)&from, &flen);
      if (n > 0) {
        buf[n] = '\0';
        // Accept either "3\n" or "c:3\n" or "3.0"
        char* p = buf;
        while (*p && (*p==' ' || *p=='\t' || *p=='c' || *p==':' )) ++p;
        double v = atof(p);
        if (v < 0.0) v = 0.0;
        if (v > 4.0) v = 4.0;
        g_class_idx.store(v, std::memory_order_relaxed);
        g_last_class_rx_s.store(now_steady_s(), std::memory_order_relaxed);

      }
    }
  }
  closesock(s);
  WSACleanup();
}

static bool parse_two_doubles(const char* s, double& a, double& b) {
  // tolerant: finds the last two numbers in the line
  // examples accepted: "123,456", "v: 0.12, -0.87", "foo,1.2,3.4,bar"
  double x[2]={0,0}; int found=0;
  const char* p = s;
  while (*p && found<2) {
    char* end=nullptr;
    double v = strtod(p, &end);
    if (end!=p) { // parsed a number
      x[found++] = v;
      p = end;
    } else {
      ++p;
    }
  }
  if (found==2) { a=x[0]; b=x[1]; return true; }
  return false;
}
// Symmetric binary framing: 0xAA 0x55 + 2 x float32 (little-endian)
static constexpr uint8_t HDR0 = 0xAA;
static constexpr uint8_t HDR1 = 0x55;

// Small helper: open + configure the COM port. Returns INVALID_HANDLE_VALUE on failure.
static HANDLE open_arduino_once() {
  HANDLE h = CreateFileA(
      ARDUINO_COM, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING,
      FILE_ATTRIBUTE_NORMAL, NULL);
  if (h == INVALID_HANDLE_VALUE) {
    return INVALID_HANDLE_VALUE;
  }

  
  DCB dcb{}; dcb.DCBlength = sizeof(DCB);
  if (!GetCommState(h, &dcb)) { CloseHandle(h); return INVALID_HANDLE_VALUE; }

  dcb.BaudRate = ARDUINO_BAUD;
  dcb.ByteSize = 8;
  dcb.Parity   = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fBinary  = TRUE;
  dcb.fDtrControl = DTR_CONTROL_ENABLE;
  dcb.fRtsControl = RTS_CONTROL_ENABLE;

  if (!SetCommState(h, &dcb)) { CloseHandle(h); return INVALID_HANDLE_VALUE; }

  COMMTIMEOUTS to{};
  to.ReadIntervalTimeout         = 1;
  to.ReadTotalTimeoutMultiplier  = 0;
  to.ReadTotalTimeoutConstant    = 10;
  SetCommTimeouts(h, &to);

  PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
  // Give CDC device time to reset the port after open
  Sleep(2000);
  EscapeCommFunction(h, CLRDTR);
  Sleep(20);
  EscapeCommFunction(h, SETDTR);


  return h;
}

static HANDLE open_teensy_once() {
  HANDLE h = CreateFileA(
      TEENSY_COM, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING,
      FILE_ATTRIBUTE_NORMAL, NULL);
  if (h == INVALID_HANDLE_VALUE) {
    return INVALID_HANDLE_VALUE;
  }

  DCB dcb{}; dcb.DCBlength = sizeof(DCB);
  if (!GetCommState(h, &dcb)) { CloseHandle(h); return INVALID_HANDLE_VALUE; }

  dcb.BaudRate = TEENSY_BAUD;
  dcb.ByteSize = 8;
  dcb.Parity   = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fBinary  = TRUE;
  dcb.fDtrControl = DTR_CONTROL_ENABLE;
  dcb.fRtsControl = RTS_CONTROL_ENABLE;

  if (!SetCommState(h, &dcb)) { CloseHandle(h); return INVALID_HANDLE_VALUE; }

  COMMTIMEOUTS to{};
  to.ReadIntervalTimeout         = 1;
  to.ReadTotalTimeoutMultiplier  = 0;
  to.ReadTotalTimeoutConstant    = 10;
  SetCommTimeouts(h, &to);

  PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
  Sleep(2000);
  EscapeCommFunction(h, CLRDTR);
  Sleep(20);
  EscapeCommFunction(h, SETDTR);

  return h;
}




void arduino_serial_thread_fn() {
  const double ARDUINO_SEND_MS = 1000.0 / ARDUINO_SEND_HZ;
  ULONGLONG last_rx_tick = GetTickCount64();

  uint8_t rxbuf[512];
  size_t  have = 0;
  HANDLE  h = INVALID_HANDLE_VALUE;
  ULONGLONG last_tx = 0;

  auto reset_buffers = [&](){
    have = 0;
    last_tx = GetTickCount64();
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

  };

  while (running.load(std::memory_order_relaxed)) {

    // --- CONNECT PHASE ---
    if (h == INVALID_HANDLE_VALUE) {
      g_serial_connected.store(false, std::memory_order_relaxed);
      std::cerr << "[arduino] Opening " << ARDUINO_COM << " ...\n";
      h = open_arduino_once();
      if (h == INVALID_HANDLE_VALUE) {
        // Could not open yet: keep trying
        std::cerr << "[arduino] Not found. Will retry in " << ARDUINO_RETRY_MS << " ms.\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(ARDUINO_RETRY_MS));
        continue;
      }
      std::cout << "[arduino] Connected " << ARDUINO_COM << " @ " << ARDUINO_BAUD << "\n";
      g_serial_connected.store(true, std::memory_order_relaxed);
      reset_buffers();
    }

    // --- IO PHASE ---
    // 1) READ frames from Arduino (tolerant, framed: 0xAA 0x55 + f32 + f32)
    DWORD n = 0;
    BOOL ok = ReadFile(h, rxbuf + have, (DWORD)(sizeof(rxbuf) - have), &n, NULL);
    if (!ok) {
      DWORD err = GetLastError();
      std::cerr << "[arduino] ReadFile failed (err=" << err << "). Reconnecting...\n";
      CloseHandle(h); h = INVALID_HANDLE_VALUE;
      continue; // loop will re-open
    }
    if (n > 0) last_rx_tick = GetTickCount64();
    have += n;
    if (have > sizeof(rxbuf)) {
      // Drop oldest half if we somehow overflowed (very defensive)
      memmove(rxbuf, rxbuf + have/2, have - have/2);
      have -= have/2;
    }

    // scan for framed packets
    auto consume = [&](size_t k){
      memmove(rxbuf, rxbuf + k, have - k);
      have -= k;
    };

    for (;;) {
      if (have < 2) break;

      size_t i = 0;
      while (i + 1 < have && !(rxbuf[i] == 0xAA && rxbuf[i+1] == 0x55)) ++i;
      if (i) { consume(i); continue; }

      // Prefer 4-float frame if available
      if (have >= 18) {
        float v1, v2, f1, f2;
        std::memcpy(&v1, rxbuf +  2, 4);
        std::memcpy(&v2, rxbuf +  6, 4);
        std::memcpy(&f1, rxbuf + 10, 4);
        std::memcpy(&f2, rxbuf + 14, 4);

        g_arduino_v1.store((double)v1, std::memory_order_relaxed);
        g_arduino_v2.store((double)v2, std::memory_order_relaxed);
        g_arduino_f1.store((double)f1, std::memory_order_relaxed);  // NEW
        g_arduino_f2.store((double)f2, std::memory_order_relaxed);  // NEW

        consume(18);
        continue;
      }

      // Fallback: legacy 2-float frame
      if (have >= 10) {
        float v1, v2;
        std::memcpy(&v1, rxbuf + 2, 4);
        std::memcpy(&v2, rxbuf + 6, 4);
        g_arduino_v1.store((double)v1, std::memory_order_relaxed);
        g_arduino_v2.store((double)v2, std::memory_order_relaxed);
        g_arduino_f1.store(NAN, std::memory_order_relaxed);         // mark missing
        g_arduino_f2.store(NAN, std::memory_order_relaxed);
        consume(10);
        continue;
      }

      // Not enough bytes yet
      break;
    }
    // 2) WRITE latest commands back to Arduino at fixed rate
    ULONGLONG now = GetTickCount64();
    if (now - last_tx >= (ULONGLONG)ARDUINO_SEND_MS) {
      last_tx = now;

      float c1 = (float)g_cmd1.load(std::memory_order_relaxed);
      float c2 = (float)g_cmd2.load(std::memory_order_relaxed);

      uint8_t frame[10];
      frame[0] = 0xAA; frame[1] = 0x55;
      std::memcpy(frame + 2, &c1, 4);
      std::memcpy(frame + 6, &c2, 4);

      DWORD nw = 0;
      if (!WriteFile(h, frame, (DWORD)sizeof(frame), &nw, NULL)) {
        DWORD err = GetLastError();
        std::cerr << "[arduino] WriteFile failed (err=" << err << "). Reconnecting...\n";
        CloseHandle(h); h = INVALID_HANDLE_VALUE;
        continue;
      }
    }
    ULONGLONG now_ms = GetTickCount64();
    // 1) If silent for >1s, send a small "sync" frame to kick Simulink serial blocks
    if (now_ms - last_rx_tick > 1000) {
        float zero = 0.0f;
        uint8_t sync[10] = {0xAA, 0x55};
        std::memcpy(sync + 2, &zero, 4);
        std::memcpy(sync + 6, &zero, 4);
        DWORD nw = 0;
        WriteFile(h, sync, (DWORD)sizeof(sync), &nw, NULL);
    }
    // 2) If still silent for >4s, force a reopen (clears stalled endpoints)
    if (now_ms - last_rx_tick > 4000) {
        std::cerr << "[arduino] RX idle >4s. Reopening...\n";
        CloseHandle(h);
        h = INVALID_HANDLE_VALUE;
        continue; // reconnect loop

    }

    // Avoid busy loop
    Sleep(1);
  }

  if (h != INVALID_HANDLE_VALUE) {
    CloseHandle(h);
  }
  g_serial_connected.store(false, std::memory_order_relaxed);
}


// Teensy framing: 0xAB 0xCD + 6 x float32 (x,y,z,theta_x,theta_y,theta_z)
static constexpr uint8_t TEENSY_HDR0 = 0xAB;
static constexpr uint8_t TEENSY_HDR1 = 0xCD;
static constexpr size_t  TEENSY_FRAME_BYTES = 2 + 6*sizeof(float);

// Python pose UDP: receives 6 x float32 = [x,y,z,mx,my,mz] on 127.0.0.1:PY_POSE_UDP_PORT
void python_pose_rx_thread_fn() {
  WSADATA wsa;
  if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) {
    std::cerr << "[pose] WSAStartup failed\n";
    return;
  }

  socket_t s = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (s == INVALID_SOCKET) {
    std::cerr << "[pose] socket() failed\n";
    WSACleanup();
    return;
  }

  sockaddr_in addr{};
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);      // listen on all local interfaces
  addr.sin_port        = htons(PY_POSE_UDP_PORT);

  if (bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) {
    std::cerr << "[pose] bind(" << PY_POSE_UDP_PORT << ") failed\n";
    closesock(s);
    WSACleanup();
    return;
  }

  std::cout << "[pose] Listening for pose on udp://127.0.0.1:" << PY_POSE_UDP_PORT << "\n";

  uint8_t buf[64];

  while (running.load(std::memory_order_relaxed)) {
    sockaddr_in from{};
    socklen_t   fromlen = sizeof(from);
    int n = recvfrom(s, (char*)buf, sizeof(buf), 0, (sockaddr*)&from, &fromlen);
    if (n < 0) {
      // non-blocking would need select(); here we just do a tiny sleep
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    if (n == 24) {
      float fx, fy, fz, fmx, fmy, fmz;
      std::memcpy(&fx,  buf +  0, 4);
      std::memcpy(&fy,  buf +  4, 4);
      std::memcpy(&fz,  buf +  8, 4);
      std::memcpy(&fmx, buf + 12, 4);
      std::memcpy(&fmy, buf + 16, 4);
      std::memcpy(&fmz, buf + 20, 4);

      g_x.store(   (double)fx,  std::memory_order_relaxed);
      g_y.store(   (double)fy,  std::memory_order_relaxed);
      g_z.store(   (double)fz,  std::memory_order_relaxed);
      g_thx.store( (double)fmx, std::memory_order_relaxed);  // repurpose thx/th y/thz as orientation vector
      g_thy.store( (double)fmy, std::memory_order_relaxed);
      g_thz.store( (double)fmz, std::memory_order_relaxed);
      g_last_pose_rx_s.store(now_steady_s(), std::memory_order_relaxed);
    } else if (n > 0) {
      // Unexpected packet size, ignore.
      std::cerr << "[pose] Ignoring packet of size " << n << "\n";
    }
  }

  closesock(s);
  WSACleanup();
}



int main() {
  std::cout << "C++ UDP TX/RX demo (ImGui+ImPlot)\n";
  std::cout << "Sending to " << LINUX_IP << ":" << SEND_PORT << ", receiving on " << RECV_PORT << "\n";

  SharedBuffers buf;
  std::thread thr_io(io_thread_fn, &buf);
  std::thread thr_echo;
  std::thread thr_class(class_rx_thread_fn);
  std::thread thr_arduino(arduino_serial_thread_fn);
  std::thread thr_pose(python_pose_rx_thread_fn);
  if (ENABLE_LOCAL_ECHO_TEST) thr_echo = std::thread(local_echo_thread_fn);

  // ===== Session recording state =====
  static bool         g_is_recording = false;
  static std::ofstream g_csv_file;
  static size_t       g_last_logged_index = 0;
  static char         g_session_name[128] = "";



  // Win32 window class & create window
  WNDCLASSEX wc = { sizeof(WNDCLASSEX), CS_CLASSDC, WndProc, 0L, 0L, GetModuleHandle(NULL), NULL, NULL, NULL, NULL, _T("UDPPlots"), NULL };
  RegisterClassEx(&wc);
  HWND hwnd = CreateWindow(wc.lpszClassName, _T("TX/RX Plots"), WS_OVERLAPPEDWINDOW, 100, 100, 1100, 800, NULL, NULL, wc.hInstance, NULL);

  // D3D11
  if (!CreateDeviceD3D(hwnd)) { CleanupDeviceD3D(); UnregisterClass(wc.lpszClassName, wc.hInstance); return 1; }
  ShowWindow(hwnd, SW_SHOWDEFAULT);
  UpdateWindow(hwnd);

  // ImGui + ImPlot setup
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGuiIO& io = ImGui::GetIO(); (void)io;
  ImGui::StyleColorsDark();
  ImGui_ImplWin32_Init(hwnd);
  ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

  struct YLim { double min, max; };
  YLim ylim_tx1_rx4{-90.0,90.0}, ylim_tx2_rx5{-0.010,0.090}, ylim_tx3_rx6{-1.0,5.0};
  YLim ylim_rx1{-3.0,3.0}, ylim_rx2{-1000.0,11000.0}, ylim_rx3{-1.2,6.0}; 
  YLim ylim_fe_wy{-1.2, 1.2}, ylim_ru_wz{-1.2, 1.2};

  // Velocity plots
  YLim ylim_vx{-1.2, 1.2};
  YLim ylim_vy{-1.2, 1.2};
  YLim ylim_vz{-1.2, 1.2};
  // NEW: channel energy E(t)
  YLim ylim_E{-50.0, 50.0};
  YLim ylim_Faxial{-20.0, 20.0};
  YLim ylim_Fradial{-20.0, 20.0};



  MSG msg; ZeroMemory(&msg, sizeof(msg));
  auto last_status = std::chrono::steady_clock::now();

  while (msg.message != WM_QUIT && running.load(std::memory_order_relaxed)) {
    while (PeekMessage(&msg, NULL, 0U, 0U, PM_REMOVE)) {
      TranslateMessage(&msg);
      DispatchMessage(&msg);
    }
    if (msg.message == WM_QUIT) break;

    ImGui_ImplDX11_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();
    // ---- Link-status values (also used for CSV logging) ----
    bool   ls_class_ok  = false;
    bool   ls_serial_ok = false;
    bool   ls_robot_ok  = false;
    bool   ls_pose_ok   = false;
    double ls_rtt_avg   = std::numeric_limits<double>::quiet_NaN();
    double ls_rtt_std   = std::numeric_limits<double>::quiet_NaN();
    double ls_rtt_ms    = std::numeric_limits<double>::quiet_NaN();  // RTT of last echo

    ImGui::Begin("Link Status");
    double now_s = now_steady_s();

    // Classifier status
    double last_class = g_last_class_rx_s.load(std::memory_order_relaxed);
    bool   class_ok   = (last_class > 0.0) &&
                        ((now_s - last_class) <= CLASS_RX_TIMEOUT_S);
    ls_class_ok = class_ok;
    ImGui::Text("Classifier UDP: %s", class_ok ? "OK" : "LOST → REST");

    // Arduino
    bool serial_ok = g_serial_connected.load(std::memory_order_relaxed);
    ls_serial_ok   = serial_ok;
    ImGui::Text("Arduino COM3:  %s",
                serial_ok ? "CONNECTED" : "CONNECTING...");

    // Robot connection based on ROBOT_RX_TIMEOUT_S
    double last_robot = g_last_robot_rx_s.load(std::memory_order_relaxed);
    double robot_age  = (last_robot < 0.0) ? 1e9 : (now_s - last_robot);
    bool   robot_ok   = (robot_age <= ROBOT_RX_TIMEOUT_S);
    ls_robot_ok = robot_ok;
    ImGui::Text("Robot UDP:     %s", robot_ok ? "CONNECTED" : "DISCONNECTED");

    // RTT latency (3 s window, avg ± std), updated every 3 s
    {
      double now_s_local = now_steady_s();
      double avg_ms = std::numeric_limits<double>::quiet_NaN();
      double std_ms = std::numeric_limits<double>::quiet_NaN();

      if (robot_ok) {
          std::lock_guard<std::mutex> lk(g_rtt_mutex);

          // Keep only last 3 s of samples
          while (!g_rtt_samples.empty() &&
                (now_s_local - g_rtt_samples.front().first) > 3.0) {
              g_rtt_samples.pop_front();
          }

          // Recompute stats at most every 3 s, if we have enough samples
          if ((now_s_local - g_rtt_last_compute_s >= 3.0) &&
              (g_rtt_samples.size() >= 2)) {

              const size_t n = g_rtt_samples.size();
              double sum = 0.0;
              for (const auto& p : g_rtt_samples) {
                  sum += p.second;
              }
              double mean = sum / double(n);

              double var = 0.0;
              for (const auto& p : g_rtt_samples) {
                  double d = p.second - mean;
                  var += d * d;
              }
              var /= double(n - 1);  // sample variance

              g_rtt_avg_ms         = mean;
              g_rtt_std_ms         = std::sqrt(var);
              g_rtt_last_compute_s = now_s_local;
          }

          avg_ms = g_rtt_avg_ms;
          std_ms = g_rtt_std_ms;
      }

      ls_rtt_avg = avg_ms;
      ls_rtt_std = std_ms;

      if (robot_ok && avg_ms == avg_ms && std_ms == std_ms) {
          ImGui::Text("RTT latency (3s): %.2f ± %.2f ms", avg_ms, std_ms);
      } else {
          ImGui::Text("RTT latency (3s): n/a");
      }
    }
    ls_rtt_ms = g_last_rtt_ms.load(std::memory_order_relaxed);

    // Python pose UDP status: based on last pose receive time
    {
        double last_pose = g_last_pose_rx_s.load(std::memory_order_relaxed);
        double pose_age  = (last_pose < 0.0) ? 1e9 : (now_s - last_pose);
        bool   pose_ok   = (pose_age <= POSE_RX_TIMEOUT_S);
        ls_pose_ok = pose_ok;

        ImGui::Text("Python pose UDP: %s", pose_ok ? "receiving" : "NO DATA");
    }

    // --- Robot pose from UDP telemetry ---
    {
        double x_udp    = g_robot_x.load(std::memory_order_relaxed);
        double z_udp    = g_robot_z.load(std::memory_order_relaxed);
        double y_udp    = g_robot_y.load(std::memory_order_relaxed);
        double roll_rad = g_robot_roll.load(std::memory_order_relaxed);
        double pitch_rad= g_robot_pitch.load(std::memory_order_relaxed);
        double yaw_rad  = g_robot_yaw.load(std::memory_order_relaxed);

        double roll_deg  = roll_rad  * 180.0 / M_PI;
        double pitch_deg = pitch_rad * 180.0 / M_PI;
        double yaw_deg   = yaw_rad   * 180.0 / M_PI;

        ImGui::Text(
            "Robot Pose: (x=%.3f, y=%.3f, z=%.3f, roll=%.1f, pitch=%.1f, yaw=%.1f)",
            x_udp, y_udp, z_udp, roll_deg, pitch_deg, yaw_deg
        );
    }

    ImGui::End();




    // Fetch data snapshot
    std::vector<double> xs, tx1, tx2, tx3, f1s, f2s;
    std::array<std::vector<double>, 3>  vel;   // vx, vy, vz
    std::array<std::vector<double>,12> rx;     // 12 telemetry channels

    {
      std::scoped_lock lk(buf.mtx);
      xs  = buf.t_hist.snapshot();
      tx1 = buf.tx_hist[0].snapshot();
      tx2 = buf.tx_hist[1].snapshot();
      tx3 = buf.tx_hist[2].snapshot();

      for (int i = 0; i < 3;  ++i) vel[i] = buf.vel_hist[i].snapshot();
      for (int i = 0; i < 12; ++i) rx[i]  = buf.rx_hist[i].snapshot();


      f1s = buf.f_hist[0].snapshot();
      f2s = buf.f_hist[1].snapshot();
    }


    // rx[5] = wy proportion (−1..+1), rx[6] = wz proportion (−1..+1)
    std::vector<double> wy_prop = rx[5];
    std::vector<double> wz_prop = rx[6];

    // Command tracks from class index (TX3 == classifier class)
    std::vector<double> fe_cmd, ru_cmd;
    fe_cmd.reserve(tx3.size());
    ru_cmd.reserve(tx3.size());
    for (double c : tx3) {
      fe_cmd.push_back( (c==1.0) ? 1.0 : (c==2.0) ? -1.0 : 0.0 ); // +1 ext, -1 flex
      ru_cmd.push_back( (c==3.0) ? 1.0 : (c==4.0) ? -1.0 : 0.0 ); // +1 radial, -1 ulnar
    }

    // ClassActive derived from proportions: threshold 0.5
    std::vector<double> class_active;
    {
      const size_t N = std::min(wy_prop.size(), wz_prop.size());
      class_active.reserve(N);
      for (size_t i=0;i<N;++i) {
        const double wy = wy_prop[i], wz = wz_prop[i];
        double act = 0.0;
        if      (wy >= +0.5) act = 1.0; // extension
        else if (wy <= -0.5) act = 2.0; // flexion
        else if (wz >= +0.5) act = 3.0; // radial
        else if (wz <= -0.5) act = 4.0; // ulnar
        class_active.push_back(act);
      }
    }

    // --- Build commanded vs measured angular velocities around Y/Z (rad/s) ---
    std::vector<double> wy_cmd_rad, wy_meas_rad, wz_cmd_rad, wz_meas_rad;
    {
      size_t Ny = std::min(wy_prop.size(), fe_cmd.size());
      wy_cmd_rad.resize(Ny);
      wy_meas_rad.resize(Ny);
      for (size_t i = 0; i < Ny; ++i) {
        wy_cmd_rad[i]  = fe_cmd[i] * W_FIXED_YZ;   // ±0.1 or 0
        wy_meas_rad[i] = wy_prop[i] * W_FIXED_YZ;  // proportion → rad/s
      }

      size_t Nz = std::min(wz_prop.size(), ru_cmd.size());
      wz_cmd_rad.resize(Nz);
      wz_meas_rad.resize(Nz);
      for (size_t i = 0; i < Nz; ++i) {
        wz_cmd_rad[i]  = ru_cmd[i] * W_FIXED_YZ;
        wz_meas_rad[i] = wz_prop[i] * W_FIXED_YZ;
      }
    }

    // // --- Scale translational velocities to m/s ---
    // for (double &v : vel[0]) v *= V_MAX_CART;  // vx_cmd
    // for (double &v : vel[1]) v *= V_MAX_CART;  // vy_cmd
    // for (double &v : vel[2]) v *= V_MAX_CART;  // vz_cmd
    // for (double &v : rx[7])  v *= V_MAX_CART;  // vx_meas
    // for (double &v : rx[8])  v *= V_MAX_CART;  // vy_meas
    // for (double &v : rx[9])  v *= V_MAX_CART;  // vz_meas

    auto plot_triple = [&](const char* win_title, const char* plot_title,
                          YLim& y,
                          const char* n1, const std::vector<double>& v1,
                          const char* n2, const std::vector<double>& v2,
                          const char* n3, const std::vector<double>& v3) {
      ImGui::Begin(win_title);
      ImGui::Text("Y-limits"); ImGui::SameLine();
      ImGui::SetNextItemWidth(100); ImGui::InputDouble("min", &y.min, 0, 0, "%.3f");
      ImGui::SameLine();
      ImGui::SetNextItemWidth(100); ImGui::InputDouble("max", &y.max, 0, 0, "%.3f");
      if (ImPlot::BeginPlot(plot_title, ImVec2(-1, -1))) {
        ImPlot::SetupAxes("Time (s)", "Value", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_None);
        ImPlot::SetupAxisLimits(ImAxis_Y1, y.min, y.max, ImGuiCond_Always);
        if (!xs.empty()) {
          ImPlot::SetupAxisLimits(ImAxis_X1, xs.front(), xs.back(), ImGuiCond_Always);
          auto npts = (int)xs.size();
          if (!v1.empty()) ImPlot::PlotLine(n1, xs.data(), v1.data(), std::min(npts,(int)v1.size()));
          if (!v2.empty()) ImPlot::PlotLine(n2, xs.data(), v2.data(), std::min(npts,(int)v2.size()));
          if (!v3.empty()) ImPlot::PlotLine(n3, xs.data(), v3.data(), std::min(npts,(int)v3.size()));
        }
        ImPlot::EndPlot();
      }
      ImGui::End();
    };


    auto plot_pair = [&](const char* win_title, const char* plot_title,
                         YLim& y, const char* name_a, const std::vector<double>& a,
                         const char* name_b, const std::vector<double>& b) {
      ImGui::Begin(win_title);
      ImGui::Text("Y-limits"); ImGui::SameLine();
      ImGui::SetNextItemWidth(100); ImGui::InputDouble("min", &y.min, 0, 0, "%.3f");
      ImGui::SameLine();
      ImGui::SetNextItemWidth(100); ImGui::InputDouble("max", &y.max, 0, 0, "%.3f");
      if (ImPlot::BeginPlot(plot_title, ImVec2(-1, -1))) {
        ImPlot::SetupAxes("Time (s)", "Value", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_None);
        ImPlot::SetupAxisLimits(ImAxis_Y1, y.min, y.max, ImGuiCond_Always);
        if (!xs.empty()) {
          ImPlot::SetupAxisLimits(ImAxis_X1, xs.front(), xs.back(), ImGuiCond_Always);
          if (!a.empty()) ImPlot::PlotLine(name_a, xs.data(), a.data(), (int)std::min(xs.size(), a.size()));
          if (!b.empty()) ImPlot::PlotLine(name_b, xs.data(), b.data(), (int)std::min(xs.size(), b.size()));
        }
        ImPlot::EndPlot();
      }
      ImGui::End();
    };

    auto plot_single = [&](const char* win_title, const char* plot_title,
                           YLim& y, const char* name, const std::vector<double>& v) {
      ImGui::Begin(win_title);
      ImGui::Text("Y-limits"); ImGui::SameLine();
      ImGui::SetNextItemWidth(100); ImGui::InputDouble("min", &y.min, 0, 0, "%.3f");
      ImGui::SameLine();
      ImGui::SetNextItemWidth(100); ImGui::InputDouble("max", &y.max, 0, 0, "%.3f");
      if (ImPlot::BeginPlot(plot_title, ImVec2(-1, -1))) {
        ImPlot::SetupAxes("Time (s)", "Value", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_None);
        ImPlot::SetupAxisLimits(ImAxis_Y1, y.min, y.max, ImGuiCond_Always);
        if (!xs.empty()) {
          ImPlot::SetupAxisLimits(ImAxis_X1, xs.front(), xs.back(), ImGuiCond_Always);
          if (!v.empty()) ImPlot::PlotLine(name, xs.data(), v.data(), (int)std::min(xs.size(), v.size()));
        }
        ImPlot::EndPlot();
      }
      ImGui::End();
    };

    // Build a smoothed copy of F1 for plotting
    std::vector<double> f1_plot;
    double dt = (xs.size() > 1) ? (xs.back() - xs.front()) / (xs.size() - 1) : 0.01; // ~10ms at 100 Hz
    double alpha = ema_alpha(2.0 /*cutoff Hz, tweak*/, dt);
    ema(f1s, alpha, f1_plot);
    // --- Channel energy E_ch(t) from torques RX1 (robot) and F1 (wrist) ---
    std::vector<double> E_ch;
    {
      const auto &tau_robot = rx[0];   // RX1: robot felt torque
      const auto &tau_wrist = f1_plot; // F1: motor torque at wrist (smoothed)
      const auto &theta_cmd_deg  = tx1;   // TX1: commanded angle (deg)
      const auto &theta_meas_deg = rx[3]; // RX4: measured angle (deg)

      size_t N = std::min(
          { xs.size(), tau_robot.size(), tau_wrist.size(),
            theta_cmd_deg.size(), theta_meas_deg.size() });

      if (N >= 2) {
        E_ch.resize(N);
        E_ch[0] = 0.0;
        const double DEG2RAD = M_PI / 180.0;

        for (size_t i = 1; i < N; ++i) {
          double dt_i = xs[i] - xs[i - 1];
          if (dt_i <= 0.0) {
            E_ch[i] = E_ch[i - 1];
            continue;
          }

          double theta_cmd_i   = theta_cmd_deg[i]   * DEG2RAD;
          double theta_cmd_im1 = theta_cmd_deg[i-1] * DEG2RAD;
          double theta_meas_i   = theta_meas_deg[i]   * DEG2RAD;
          double theta_meas_im1 = theta_meas_deg[i-1] * DEG2RAD;

          double omega_cmd  = (theta_cmd_i  - theta_cmd_im1)  / dt_i; // wrist / operator side
          double omega_meas = (theta_meas_i - theta_meas_im1) / dt_i; // robot side

          double P_ch = tau_wrist[i] * omega_cmd    // operator power
                      - tau_robot[i] * omega_meas;  // minus robot power

          E_ch[i] = E_ch[i - 1] + P_ch * dt_i;      // rectangular integration
        }
      }
    }

        // ------- Session Recorder GUI -------

    ImGui::Begin("Session Recorder");
    ImGui::InputText("Session name", g_session_name, IM_ARRAYSIZE(g_session_name));

    if (!g_is_recording) {
        if (ImGui::Button("Start recording")) {
            if (std::strlen(g_session_name) > 0) {
                try {
                    std::filesystem::create_directory("sessions");
                } catch (...) {
                    // ignore if it already exists or cannot be created
                }

                std::string filename = std::string("sessions/") + g_session_name + ".csv";
                g_csv_file.open(filename, std::ios::out);
                if (g_csv_file.is_open()) {
                    g_is_recording = true;

                    // Use time-based logging. Start from "now": only log samples with t > current last time.
                    if (!xs.empty()) {
                        g_last_logged_time = xs.back();
                    } else {
                        g_last_logged_time = -1.0;
                    }

                    g_csv_file << std::fixed << std::setprecision(6);

                    // Header: comma-separated
                    g_csv_file
                        << "timestamp"
                        << ",phi_x_cmd_deg"   // TX1
                        << ",phi_x_meas_deg"  // RX4
                        << ",g_cmd_m"         // TX2
                        << ",g_meas_m"        // RX5
                        << ",wy_cmd_rad_s"
                        << ",wy_meas_rad_s"
                        << ",wz_cmd_rad_s"
                        << ",wz_meas_rad_s"
                        << ",vx_cmd"
                        << ",vx_meas"
                        << ",vy_cmd"
                        << ",vy_meas"
                        << ",vz_cmd"
                        << ",vz_meas"
                        << ",TX3_class"
                        << ",ClassActive"
                        << ",RX1_tau_toolx"
                        << ",F1_wrist"
                        << ",RX2"
                        << ",F2"
                        << ",RX3_hstate"
                        << ",E_ch"
                        << ",robot_x"
                        << ",robot_y"
                        << ",robot_z"
                        << ",robot_roll_deg"
                        << ",robot_pitch_deg"
                        << ",robot_yaw_deg"
                        << ",class_ok"
                        << ",serial_ok"
                        << ",robot_ok"
                        << ",pose_ok"
                        << ",rtt_ms"
                        << ",linux_ip"
                        << ",F_axial_toolx_N"
                        << ",F_radial_toolYZ_N"
                        << "\n";

                }
            }
        }
    } else {
        ImGui::Text("Recording: ON");
        if (ImGui::Button("Stop recording")) {
            g_is_recording = false;
            if (g_csv_file.is_open()) {
                g_csv_file.close();
            }
        }
    }
    ImGui::End();

        // ------- CSV logging of all displayed data -------

    if (g_is_recording && g_csv_file.is_open() && !xs.empty()) {
        // Current robot pose (latest values, will be repeated across samples)
        double robot_x    = g_robot_x.load(std::memory_order_relaxed);
        double robot_y    = g_robot_y.load(std::memory_order_relaxed);
        double robot_z    = g_robot_z.load(std::memory_order_relaxed);
        double robot_roll = g_robot_roll.load(std::memory_order_relaxed);
        double robot_pitch= g_robot_pitch.load(std::memory_order_relaxed);
        double robot_yaw  = g_robot_yaw.load(std::memory_order_relaxed);

        const double RAD2DEG = 180.0 / M_PI;
        double robot_roll_deg  = robot_roll  * RAD2DEG;
        double robot_pitch_deg = robot_pitch * RAD2DEG;
        double robot_yaw_deg   = robot_yaw   * RAD2DEG;

        size_t N = xs.size();

        auto get_val = [](const std::vector<double>& v, size_t i) {
            return (i < v.size()) ? v[i] : std::numeric_limits<double>::quiet_NaN();
        };

        // Find first index with t > g_last_logged_time
        size_t start_idx = 0;
        while (start_idx < N && xs[start_idx] <= g_last_logged_time) {
            ++start_idx;
        }

        for (size_t i = start_idx; i < N; ++i) {
            double t            = xs[i];

            double phi_x_cmd    = get_val(tx1, i);
            double phi_x_meas   = get_val(rx[3], i);
            double g_cmd        = get_val(tx2, i);
            double g_meas       = get_val(rx[4], i);

            double wy_cmd       = get_val(wy_cmd_rad, i);
            double wy_meas      = get_val(wy_meas_rad, i);
            double wz_cmd       = get_val(wz_cmd_rad, i);
            double wz_meas      = get_val(wz_meas_rad, i);

            double vx_cmd       = get_val(vel[0], i);
            double vy_cmd       = get_val(vel[1], i);
            double vz_cmd       = get_val(vel[2], i);

            double vx_meas      = get_val(rx[7], i);
            double vy_meas      = get_val(rx[8], i);
            double vz_meas      = get_val(rx[9], i);

            double tx3_class    = get_val(tx3, i);
            double class_act    = get_val(class_active, i);
            double rx1_tau      = get_val(rx[0], i);
            double f1_wrist     = get_val(f1_plot, i);
            double rx2_val      = get_val(rx[1], i);
            double f2_val       = get_val(f2s, i);
            double rx3_hstate   = get_val(rx[2], i);
            double E_val        = get_val(E_ch, i);
            double F_axial_val  = get_val(rx[10], i);
            double F_radial_val = get_val(rx[11], i);

            double class_ok_val  = ls_class_ok  ? 1.0 : 0.0;
            double serial_ok_val = ls_serial_ok ? 1.0 : 0.0;
            double robot_ok_val  = ls_robot_ok  ? 1.0 : 0.0;
            double pose_ok_val   = ls_pose_ok   ? 1.0 : 0.0;

            g_csv_file
            << t
            << "," << phi_x_cmd
            << "," << phi_x_meas
            << "," << g_cmd
            << "," << g_meas
            << "," << wy_cmd
            << "," << wy_meas
            << "," << wz_cmd
            << "," << wz_meas
            << "," << vx_cmd
            << "," << vx_meas
            << "," << vy_cmd
            << "," << vy_meas
            << "," << vz_cmd
            << "," << vz_meas
            << "," << tx3_class
            << "," << class_act
            << "," << rx1_tau
            << "," << f1_wrist
            << "," << rx2_val
            << "," << f2_val
            << "," << rx3_hstate
            << "," << E_val
            << "," << robot_x
            << "," << robot_y
            << "," << robot_z
            << "," << robot_roll_deg
            << "," << robot_pitch_deg
            << "," << robot_yaw_deg
            << "," << class_ok_val
            << "," << serial_ok_val
            << "," << robot_ok_val
            << "," << pose_ok_val
            << "," << ls_rtt_ms
            << "," << LINUX_IP
            << "," << F_axial_val
            << "," << F_radial_val
            << "\n";
        }

        if (!xs.empty()) {
            g_last_logged_time = xs.back();
        }

        g_csv_file.flush();
    }




    // ===== 7 windows =====
    plot_pair("TX1 & RX4", "TX1 + RX4", ylim_tx1_rx4, "TX1", tx1, "RX4", rx[3]);
    plot_pair("TX2 & RX5", "TX2 + RX5", ylim_tx2_rx5, "TX2", tx2, "RX5", rx[4]);

    plot_pair("TX3 & ClassActive",
              "TX3 (class) + ClassActive (|prop|>=0.5)",
              ylim_tx3_rx6,
              "TX3", tx3,
              "ClassActive", class_active);


    plot_pair("RX1 & F1", "RX1 + F1", ylim_rx1, "RX1", rx[0], "F1", f1_plot);
    plot_pair("RX2 & F2", "RX2 + F2", ylim_rx2, "RX2", rx[1], "F2", f2s);
    plot_single("RX3", "RX3", ylim_rx3, "RX3", rx[2]);

    plot_pair("wy cmd vs meas", "wy_cmd vs wy_meas",
              ylim_fe_wy,
              "wy_cmd (rad/s)",  wy_cmd_rad,
              "wy_meas (rad/s)", wy_meas_rad);

    plot_pair("wz cmd vs meas", "wz_cmd vs wz_meas",
              ylim_ru_wz,
              "wz_cmd (rad/s)",  wz_cmd_rad,
              "wz_meas (rad/s)", wz_meas_rad);


    // ===== Velocity windows: command vs measured (tool-frame) =====
    // vel[0..2]   = commanded vx,vy,vz (from compute_velocity)
    // rx[7..9]    = measured vx,vy,vz in tool frame from robot packet

    plot_pair("vx cmd vs meas", "vx_cmd vs vx_meas",
              ylim_vx,
              "vx_cmd",  vel[0],
              "vx_meas", rx[7]);

    plot_pair("vy cmd vs meas", "vy_cmd vs vy_meas",
              ylim_vy,
              "vy_cmd",  vel[1],
              "vy_meas", rx[8]);

    plot_pair("vz cmd vs meas", "vz_cmd vs vz_meas",
              ylim_vz,
              "vz_cmd",  vel[2],
              "vz_meas", rx[9]);

        // ===== Force windows: F_axial and F_radial from robot telemetry =====
    // rx[10] = F_axial along tool-X [N]
    // rx[11] = F_radial in tool Y–Z plane [N]
    plot_single("F_axial", "F_axial along tool-X [N]",
                ylim_Faxial,
                "F_axial", rx[10]);

    plot_single("F_radial", "F_radial in tool YZ-plane [N]",
                ylim_Fradial,
                "F_radial", rx[11]);



    // ------- Channel Energy window -------
    ImGui::Begin("Channel Energy E(t)");
    ImGui::Text("Y-limits"); ImGui::SameLine();
    ImGui::SetNextItemWidth(100); ImGui::InputDouble("min", &ylim_E.min, 0, 0, "%.3f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100); ImGui::InputDouble("max", &ylim_E.max, 0, 0, "%.3f");

    if (ImPlot::BeginPlot("E_ch(t)", ImVec2(-1, -1))) {
      ImPlot::SetupAxes("Time (s)", "Energy (arb. units)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_None);
      ImPlot::SetupAxisLimits(ImAxis_Y1, ylim_E.min, ylim_E.max, ImGuiCond_Always);

      if (!xs.empty() && !E_ch.empty()) {
        int Nplot = (int)std::min(xs.size(), E_ch.size());
        ImPlot::SetupAxisLimits(ImAxis_X1, xs.front(), xs[Nplot - 1], ImGuiCond_Always);
        ImPlot::PlotLine("E_ch", xs.data(), E_ch.data(), Nplot);
      }
      ImPlot::EndPlot();
    }
    ImGui::End();


    // Status (console, once per second)
    auto now = std::chrono::steady_clock::now();
    if (now - last_status > std::chrono::seconds(1)) {
      last_status = now;
      std::array<double,3>  tx_latest{NAN,NAN,NAN};
      std::array<double,12> rx_latest{NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN};
      double t_latest = NAN;
      {
        std::scoped_lock lk(buf.mtx);
        if (!buf.t_hist.empty()) {
          t_latest = buf.t_hist.back();
          for (int i=0;i<3;  ++i)
            if (!buf.tx_hist[i].empty()) tx_latest[i] = buf.tx_hist[i].back();
          for (int i=0;i<12; ++i)
            if (!buf.rx_hist[i].empty()) rx_latest[i] = buf.rx_hist[i].back();
        }
      }
      std::cout.setf(std::ios::fixed); std::cout.precision(3);
      std::cout << "[t=" << t_latest << " s] TX=("
          << tx_latest[0] << "," << tx_latest[1] << "," << tx_latest[2] << ") RX=("
          << rx_latest[0]  << "," << rx_latest[1]  << "," << rx_latest[2]  << ","
          << rx_latest[3]  << "," << rx_latest[4]  << "," << rx_latest[5]  << ","
          << rx_latest[6]  << "," << rx_latest[7]  << "," << rx_latest[8]  << ","
          << rx_latest[9]  << "," << rx_latest[10] << "," << rx_latest[11] << ")\n";
    }

    // Render
    ImGui::Render();
    const float clear_color_with_alpha[4] = {0.1f, 0.11f, 0.12f, 1.0f};
    g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, NULL);
    g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, clear_color_with_alpha);
    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
    g_pSwapChain->Present(1, 0);
  }
  // After UI loop ends, signal threads to stop:
  running.store(false, std::memory_order_relaxed);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));


  thr_io.join();
  if (thr_echo.joinable())    thr_echo.join();
  if (thr_class.joinable())   thr_class.join();
  if (thr_arduino.joinable()) thr_arduino.join();
  if (thr_pose.joinable())  thr_pose.join();



  ImGui_ImplDX11_Shutdown();
  ImGui_ImplWin32_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  CleanupDeviceD3D();
  DestroyWindow(hwnd);
  UnregisterClass(_T("UDPPlots"), GetModuleHandle(NULL));
  return 0;
}

// Dear ImGui Win32 backend needs this symbol in a TU:
#ifdef _MSC_VER
#pragma comment(linker, "/SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")
#endif

