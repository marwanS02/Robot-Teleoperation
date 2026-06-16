// teleop_cart_ang_reply_xasJ7.cpp
// Cartesian teleop with gripper thread + poll-reply,
// classifier → tool-Y/Z angular commands, and continuous J7 approximation on tool-X.
// EE frame is rotated so tool-X aligns with gripper forward (old tool-Z / J7 axis).
//
// Build: g++ -std=c++17 teleop_cart_ang_reply_xasJ7.cpp -o teleop -lfranka -lpthread
// Run:   ./teleop <robot-ip-or-hostname>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <initializer_list>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>

#include "examples_common.h"  // MotionGenerator, setDefaultBehavior (your existing helper)

using std::cout;
using std::endl;
#define DEBUG false

// =================== CONFIG ===================
//static const char* WINDOWS_IP = "100.118.89.87";  // poll-reply destination long distance Marwan
static const char* WINDOWS_IP = "100.66.145.56";  // poll-reply destination long distance Huimin
//static const char* WINDOWS_IP = "192.168.4.2";  // poll-reply destination short distance Marwan
//static const char* WINDOWS_IP = "192.168.4.3";  // poll-reply destination short distance Huimin

static const int   DATA_RECV_PORT = 5005;         // incoming: joint_rot_deg, bend_raw, class_id (3 doubles)
static const int   POLL_RECV_PORT = 5007;         // incoming ping (24 bytes) to trigger reply
static const int   SEND_PORT      = 5006;         // reply port (6 doubles)

// How long we tolerate no data from PC before forcing zero motion [ms]
static constexpr uint64_t DATA_TIMEOUT_MS = 200;  // 0.2 s


// ---- VIRTUAL BORDERS (Cartesian + Orientation) ----
// Base-frame Cartesian position limits [m]
static constexpr double X_MIN = 0.20, X_MAX = 0.90;
static constexpr double Y_MIN = -0.70, Y_MAX = 0.70;
static constexpr double Z_MIN = -0.40, Z_MAX = 0.70;

// Near-border distances [m] and [rad]
static constexpr double NEAR_D_POS = 0.1;    // within 6 cm → NEAR
static constexpr double NEAR_D_ANG = 4.0 * M_PI/180.0;  // within 8° → NEAR

// Orientation limits as Euler ZYX (yaw, pitch, roll) in radians
// (Adjust to your workspace needs.)
// Orientation limits as Euler ZYX (yaw, pitch, roll) in radians
static constexpr double YAW_MIN   = -190.0 * M_PI/180.0;
static constexpr double YAW_MAX   = +190.0 * M_PI/180.0;
static constexpr double PITCH_MIN = -190.0 * M_PI/180.0;
static constexpr double PITCH_MAX = +190.0 * M_PI/180.0;
static constexpr double ROLL_MIN  = -190.0 * M_PI/180.0;
static constexpr double ROLL_MAX  = +190.0 * M_PI/180.0;

// ===== JOINT LIMITS (Panda) =====
static constexpr std::array<double,7> Q_MIN = {
  -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973
};

static constexpr std::array<double,7> Q_MAX = {
   2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973
};

// “Near” band for joints (~10°)
static constexpr double JOINT_NEAR = 0.0 * M_PI / 180.0;

// Class → tool angular speed
static constexpr double W_FIXED_YZ  = 0.1;  // rad/s
static constexpr double W_MAX_ANG   = 0.1;  // rad/s cap
static constexpr double ALPHA_MAX   = 0.3;  // rad/s^2 accel cap
// NEW: dedicated tool-X limits
static constexpr double W_MAX_X     = 1.5;  // rad/s   max speed for wx_tool
static constexpr double ALPHA_MAX_X = 3.0;  // rad/s^2 max accel for wx_tool


// J7 approximation on tool-X (after EE rotation)
static constexpr double KPX = 2.3;          // [1/s]
static constexpr double KDX = 0.1;          // [dimensionless-ish], acts on dq7

// Linear motion kept disabled (0). Limits here for future use.
static constexpr double V_MAX_CART  = 0.03;  // m/s
static constexpr double A_MAX_CART  = 0.4;  // m/s^2


// Gripper mapping (bend_raw → width)
static constexpr double BEND_MIN_RAW = 220.0;
static constexpr double BEND_MAX_RAW = 450.0;
static constexpr double GRIPPER_MIN  = 0.000;   // m
static constexpr double GRIPPER_MAX  = 0.080;   // m
static constexpr double GRIP_SPEED   = 0.08;    // m/s
static constexpr double WIDTH_DEAD   = 0.003;   // m threshold
// ---------- Gripper control state ----------
constexpr double W_MIN = 0.000;    // m
constexpr double W_MAX = 0.080;    // m
constexpr double CLOSE_THRESH = 0.0544; // m (≈ 50/90*0.08): "closing" intent
constexpr double EPS_W  = 0.002;   // m: deadband for state decisions (2 mm)
constexpr double HOLD_BIAS = 0.001; // m: maintain squeeze slightly tighter
constexpr double MOVE_SPEED = 0.1; // m/s
constexpr double FEEDBACK_REF = 0.0444; // m maps to 4000 counts
constexpr double FEEDBACK_MAX = 2900.0;

double feedback2 = 0.0; 



// =================== SHARED STATE ===================
volatile double g_joint_rot_deg = 0.0; // desired J7 angle in degrees (from exoglove)
volatile double g_class_id      = 0.0; // {0..4}
// After (GRIPPER_MAX is already defined above):
volatile double g_bend_raw      = GRIPPER_MAX; // start as fully open
// Torque and angle around tool-X (EE frame)
volatile double g_tau_toolx     = 0.0; // torque about tool-X [Nm], from external wrench
volatile double g_wx_tool_deg   = 0.0; // roll angle about tool-X [deg], 0° ≙ physical 45°
volatile double g_grip_width    = 0.0; // gripper width (m)
volatile double g_grasp_flag    = 0.0; // 1.0 if grasped
volatile double g_collision_flag= 0.0; // 1.0 if any contact/collision
volatile double g_feedback = 0.0; // old-style scaled width feedback
// External force metrics (tool frame)
volatile double g_F_axial  = 0.0;  // [N] force along tool-X (peg axis)
volatile double g_F_radial = 0.0;  // [N] magnitude in Y–Z plane of tool frame

volatile double g_wy_prop = 0.0;  // proportion of W_FIXED_YZ actually commanded on tool-Y
volatile double g_wz_prop = 0.0;  // proportion of W_FIXED_YZ actually commanded on tool-Z
// Linear tool-frame velocity commands from PC (vx, vy, vz)
volatile double g_vx_tool = 0.0;  // [m/s]
volatile double g_vy_tool = 0.0;  // [m/s]
volatile double g_vz_tool = 0.0;  // [m/s]

// Measured / actual tool-frame velocities (from Jacobian * dq)
volatile double g_vx_tool_meas = 0.0;  // [m/s]
volatile double g_vy_tool_meas = 0.0;  // [m/s]
volatile double g_vz_tool_meas = 0.0;  // [m/s]

volatile double g_wx_tool_meas = 0.0;  // [rad/s]
volatile double g_wy_tool_meas = 0.0;  // [rad/s]
volatile double g_wz_tool_meas = 0.0;  // [rad/s]

volatile double g_phi_x_tool = 0.0; // [rad] rotation about tool-X (relative to start)
volatile double g_phi_y_tool = 0.0; // [rad] rotation about tool-Y
volatile double g_phi_z_tool = 0.0; // [rad] rotation about tool-Z

// Pose in base frame (meters)
volatile double g_px = 0.0;
volatile double g_py = 0.0;
volatile double g_pz = 0.0;

// Orientation (roll, pitch, yaw) in radians, ZYX convention
volatile double g_roll  = 0.0;
volatile double g_pitch = 0.0;
volatile double g_yaw   = 0.0;

std::mutex g_log_mutex;


// Haptic state constants (match Feather)
enum : int { H_FAR=1, H_NEAR=2, H_BORDER=3, H_GRASP=4 };
// after other globals
static std::atomic<int> g_border_state{H_FAR};

// NEW: last time we received a packet from the Windows PC (ms)
static std::atomic<uint64_t> g_last_data_ms{0};
// When > now_ms, we must output GRASP pulse (overrides borders)
static std::atomic<uint64_t> g_grasp_pulse_until_ms{0};

// Helper to read steady_clock in ms
static inline uint64_t now_ms() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}



std::atomic<bool> running{true};
// =================== HELPERS ===================
inline double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(x, hi));
}
inline double deg2rad(double d){ return d * M_PI / 180.0; }
inline double rad2deg(double r){ return r * 180.0 / M_PI; }  // <-- add this

// Smoothly kill outward velocity as we approach positional borders in base frame.
inline double soft_gate_axis(double p, double v,
                             double pmin, double pmax,
                             double soft_zone) {
  // No soft zone configured
  if (soft_zone <= 0.0) return v;

  // If we are already beyond the limits, forbid further outward motion,
  // still allow motion back in.
  if (p <= pmin && v < 0.0) return 0.0;
  if (p >= pmax && v > 0.0) return 0.0;

  // Inside [pmin, pmax]: only scale velocity *toward* the border.
  if (v > 0.0) {
    // Moving + toward pmax
    double d = pmax - p;         // distance to upper wall
    if (d >= soft_zone) return v;
    double alpha = d / soft_zone; // 1 at inner edge, 0 at wall
    return alpha * v;
  } else if (v < 0.0) {
    // Moving – toward pmin
    double d = p - pmin;         // distance to lower wall
    if (d >= soft_zone) return v;
    double alpha = d / soft_zone;
    return alpha * v;
  }

  // v == 0 → nothing to do
  return v;
}
static constexpr double POS_SOFT_ZONE = NEAR_D_POS;  // e.g. 0.06 m

// Smoothly kill outward angular velocity as we approach orientation borders
// in *tool* frame around each axis (φx, φy, φz).
inline double soft_gate_angle(double phi, double w,
                              double phi_min, double phi_max,
                              double soft_zone) {
  if (soft_zone <= 0.0) return w;

  // Already beyond limits: forbid further outward rotation,
  // still allow rotation back toward the interior.
  if (phi <= phi_min && w < 0.0) return 0.0;
  if (phi >= phi_max && w > 0.0) return 0.0;

  // Inside [phi_min, phi_max]: only scale velocity *toward* the border.
  if (w > 0.0) {
    // Rotating + toward phi_max
    double d = phi_max - phi;        // distance to upper limit
    if (d >= soft_zone) return w;
    double alpha = d / soft_zone;    // 1 at inner edge, 0 at limit
    return alpha * w;
  } else if (w < 0.0) {
    // Rotating – toward phi_min
    double d = phi - phi_min;        // distance to lower limit
    if (d >= soft_zone) return w;
    double alpha = d / soft_zone;
    return alpha * w;
  }

  return w;  // w == 0
}

static constexpr double ANG_SOFT_ZONE = NEAR_D_ANG;  // e.g. 8° in rad


// =================== NET UTILS ===================
static void set_nonblock(int fd, bool nb) {
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) return;
  if (nb) fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  else    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
}

// =================== THREADS ===================

// RX: (joint_rot_deg, bend_raw, class_id, vx, vy, vz)
// RX: (joint_rot_deg, bend_raw, class_id, vx_prop, vy_prop, vz_prop)
static void data_receiver_thread() {
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) { perror("socket data_rx"); return; }

  sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_port = htons(DATA_RECV_PORT);
  addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(sockfd, (sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("bind data_rx");
    close(sockfd);
    return;
  }
  set_nonblock(sockfd, true);
  cout << "[NET] Data RX bound on port " << DATA_RECV_PORT << endl;

while (running.load()) {
  double buf[6];
  ssize_t n = recv(sockfd, buf, sizeof(buf), 0);

  if (n == (ssize_t)sizeof(buf)) {
    bool ok = true;
    for (int i = 0; i < 6; ++i) {
      if (!std::isfinite(buf[i])) {
        ok = false;
        break;
      }
    }
    if (!ok) {
      // ignore this packet
      continue;
    }

    g_joint_rot_deg = buf[0] + 0.0;
    g_bend_raw      = buf[1];
    g_class_id      = buf[2];

    double vx_prop = clamp(buf[3], -1.0, 1.0);
    double vy_prop = clamp(buf[4], -1.0, 1.0);
    double vz_prop = clamp(buf[5], -1.0, 1.0);

    g_vx_tool = vx_prop * V_MAX_CART;
    g_vy_tool = vy_prop * V_MAX_CART;
    g_vz_tool = -vz_prop * V_MAX_CART;

    g_last_data_ms.store(now_ms(), std::memory_order_relaxed);
  } else {
    // n < 0 (EAGAIN) or short/garbage → just back off a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}


  close(sockfd);
}



// Classify one scalar vs [min,max] with "near" band
enum class Prox { FAR, NEAR, HIT };
static inline Prox classify_scalar(double v, double vmin, double vmax, double near_d) {
  if (v < vmin || v > vmax) return Prox::HIT;
  const double dmin = v - vmin, dmax = vmax - v;
  return (dmin < near_d || dmax < near_d) ? Prox::NEAR : Prox::FAR;
}

// Extract ZYX Euler from rotation matrix (column-major 3x3 in O_T_EE)
static inline void rot_to_euler_zyx(const std::array<double,16>& T,
                                    double& yaw, double& pitch, double& roll) {
  // Correct extraction
  const double r00 = T[0];
  const double r01 = T[4];
  const double r02 = T[8];
  const double r10 = T[1];
  const double r11 = T[5];
  const double r12 = T[9];
  const double r20 = T[2];
  const double r21 = T[6];
  const double r22 = T[10];

  yaw   = std::atan2(r10, r00);
  pitch = std::asin(-r20);
  roll  = std::atan2(r21, r22);
}


// Combine many Prox → haptic state (ignore FARs, NEAR if any NEAR, else BORDER if any HIT)
static inline int border_state_from_prox(const std::initializer_list<Prox>& L) {
  bool any_hit=false, any_near=false;
  for (auto p: L) { if (p==Prox::HIT) any_hit=true; else if (p==Prox::NEAR) any_near=true; }
  if (any_hit) return H_BORDER;
  if (any_near) return H_NEAR;
  return H_FAR;
}


// POLLED reply thread (6 doubles + latency echo)
static void poll_reply_thread() {
  int rx = socket(AF_INET, SOCK_DGRAM, 0);
  if (rx < 0) { perror("socket poll_rx"); return; }

  sockaddr_in addr{};
  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(POLL_RECV_PORT);
  addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(rx, (sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("bind poll_rx");
    close(rx);
    return;
  }
  set_nonblock(rx, true);

  int tx = socket(AF_INET, SOCK_DGRAM, 0);
  if (tx < 0) {
    perror("socket poll_tx");
    close(rx);
    return;
  }

  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_port   = htons(SEND_PORT);
  inet_pton(AF_INET, WINDOWS_IP, &dest.sin_addr);

  cout << "[NET] Poll RX " << POLL_RECV_PORT
       << " → Reply to " << WINDOWS_IP << ":" << SEND_PORT << endl;

  char         ping[24];
  sockaddr_in  sender{};
  socklen_t    slen = sizeof(sender);

  while (running.load()) {
    ssize_t n = recvfrom(rx, ping, sizeof(ping), 0,
                         (sockaddr*)&sender, &slen);

    const uint64_t until   = g_grasp_pulse_until_ms.load(std::memory_order_relaxed);
    const int      border  = g_border_state.load(std::memory_order_relaxed);
    const int      hstate  = (now_ms() < until) ? H_GRASP : border;

    if (n == 24) {
      // --------------------------------------------------------------------
      // 1) Extract timestamp from first double (sent by Windows side)
      // --------------------------------------------------------------------
      double ts = 0.0;
      static_assert(sizeof(double) == 8, "double must be 8 bytes");
      std::memcpy(&ts, ping + 0, sizeof(double));

      // --------------------------------------------------------------------
      // 2) Send 24-byte echo with the same timestamp (for RTT measurement)
      // --------------------------------------------------------------------
      double echo_vals[3] = { ts, 0.0, 0.0 };
      uint8_t echo_buf[24];
      std::memcpy(echo_buf + 0,  &echo_vals[0], 8);
      std::memcpy(echo_buf + 8,  &echo_vals[1], 8);
      std::memcpy(echo_buf + 16, &echo_vals[2], 8);

      ssize_t sent_echo = sendto(tx,
                                 (const char*)echo_buf,
                                 sizeof(echo_buf),
                                 0,
                                 (sockaddr*)&dest,
                                 sizeof(dest));
      if (sent_echo != (ssize_t)sizeof(echo_buf)) {
        perror("sendto echo");
      }

      // --------------------------------------------------------------------
      // 3) Normal 10-double telemetry packet (unchanged from your code)
      // --------------------------------------------------------------------
      auto clip = [](double v) {
        return (v < -1.0) ? -1.0 : (v > 1.0 ? 1.0 : v);
      };

      double vx_prop_meas = 0.0;
      double vy_prop_meas = 0.0;
      double vz_prop_meas = 0.0;

      if (V_MAX_CART > 1e-9) {
        vx_prop_meas = clip(g_vx_tool_meas / V_MAX_CART);
        vy_prop_meas = clip(g_vy_tool_meas / V_MAX_CART);
        vz_prop_meas = clip(-g_vz_tool_meas / V_MAX_CART);
      }
      // Copy pose/orientation atomically enough for our purposes
      const double px    = g_px;
      const double py    = g_py;
      const double pz    = g_pz;
      const double roll  = g_roll;   // rad
      const double pitch = g_pitch;  // rad
      const double yaw   = g_yaw;    // rad

      double pkt[18] = {
        -g_tau_toolx/1.2,   //  0: torque about tool-X [Nm] (scaled as before)
        // feedback2/2.0,
        -g_feedback,        //  1: legacy width-based feedback
        (double)hstate,    //  2: haptic state (FAR/NEAR/BORDER/GRASP)
        g_wx_tool_deg,     //  3: tool-X angle [deg] (0 ≙ 45° physical)
        g_grip_width,      //  4: gripper width [m]
        g_wy_prop,         //  5: measured wy / wy_max
        g_wz_prop,         //  6: measured wz / wz_max
        vx_prop_meas,      //  7: measured v_x (tool frame) in [-1,1]
        vy_prop_meas,      //  8: measured v_y (tool frame) in [-1,1]
        vz_prop_meas,      //  9: measured v_z (tool frame) in [-1,1]

        // Pose
        px,                // 10: x [m]
        pz,                // 11: z [m]
        py,                // 12: y [m]
        roll,              // 13: roll  [rad]
        pitch,             // 14: pitch [rad]
        yaw,               // 15: yaw   [rad]

        // New: force metrics
        g_F_axial,         // 16: axial force along tool-X [N]
        g_F_radial         // 17: radial force magnitude in tool Y–Z plane [N]
      };


      ssize_t sent = sendto(tx,
                            pkt,
                            sizeof(pkt),
                            0,
                            (sockaddr*)&dest,
                            sizeof(dest));
      if (sent != (ssize_t)sizeof(pkt)) {
        perror("sendto reply");
      }


    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  close(tx);
  close(rx);
}


// Robust binary gripper control using measured contact width:
// 1) Finger "close" (for some time)  -> CLOSE sequence:
//    - move(W_MIN)  until contact / hard stop
//    - readOnce()   to get contact width
//    - grasp(width_contact, ...) to hold strongly at that width
// 2) Finger "open" (for some time)   -> OPEN: move(W_MAX)
// No logic depends on is_grasped. Only the bend sensor decides open/close.

static void gripper_thread(const std::string robot_ip) {
  constexpr double W_MIN        = ::W_MIN;        // 0.0
  constexpr double W_MAX        = ::W_MAX;        // 0.08
  constexpr double CLOSE_THRESH = ::CLOSE_THRESH; // bend-based "closed" threshold (mapped to m)
  constexpr double EPS_W        = ::EPS_W;        // ~0.002 m
  constexpr double MOVE_SPEED   = ::MOVE_SPEED;   // 0.05 m/s
  constexpr double FB_REF       = ::FEEDBACK_REF; // 0.0444 m -> 4000
  constexpr double FB_MAX       = ::FEEDBACK_MAX; // 16000

  // Finger thresholds:
  const double CLOSE_ENTER = CLOSE_THRESH - 1.0 * EPS_W;  // become "closed" below this
  const double OPEN_ENTER  = W_MAX - 1.0 * EPS_W;        // require almost fully open to reopen

  // Timing:
  const int LOOP_DT_MS       = 20;
  const int CLOSE_HOLD_MS    = 60;   // require 150 ms of "closed"
  const int OPEN_HOLD_MS     = 60;   // require 150 ms of "open"
  const int LOCKOUT_AFTER_MS = 150;   // ignore opposite command for 400 ms after a switch

  enum class GripMode { OPEN, HOLD };
  GripMode mode = GripMode::OPEN;

  int close_ms   = 0;
  int open_ms    = 0;
  int lockout_ms = 0;

  try {
    franka::Gripper gripper(robot_ip);
    // IMPORTANT: do a homing *once* somewhere in your startup code, not in this loop:
    gripper.homing();

    while (running.load()) {
      // 1) Bend from PC: must already be mapped to [0, 0.08] m.
      double cmd_w = clamp(g_bend_raw, W_MIN, W_MAX);

      bool raw_close = (cmd_w < CLOSE_ENTER);
      bool raw_open  = (cmd_w > OPEN_ENTER);

      // Lockout countdown
      if (lockout_ms > 0) {
        lockout_ms -= LOOP_DT_MS;
        if (lockout_ms < 0) lockout_ms = 0;
      }

      // Time hysteresis
      if (raw_close) {
        close_ms += LOOP_DT_MS;
      } else {
        close_ms = 0;
      }

      if (raw_open) {
        open_ms += LOOP_DT_MS;
      } else {
        open_ms = 0;
      }

      // 2) CLOSE sequence: OPEN -> HOLD
      if (mode == GripMode::OPEN &&
          lockout_ms == 0 &&
          close_ms >= CLOSE_HOLD_MS) {
        close_ms   = 0;
        lockout_ms = LOCKOUT_AFTER_MS;

        try {
          // Stage 1: close in position control until contact / hard stop
          gripper.move(W_MIN, MOVE_SPEED);

          // Stage 2: measure actual width at contact
          franka::GripperState gs_contact = gripper.readOnce();
          double contact_w = gs_contact.width;

          // Stage 3: do a force-based grasp *at that width*
          //  - If no object: contact_w ≈ W_MIN, still fine.
          //  - If small object: contact_w small.
          //  - If large object: contact_w large.
          // For all, we grasp around the real contact point.
          const double eps   = 0.005;   // ±5 mm tolerance around contact width
          const double force = 60.0;    // N, tune as needed

          // If contact_w is slightly outside [W_MIN, W_MAX], clamp it.
          contact_w = clamp(contact_w, W_MIN, W_MAX);

          (void)gripper.grasp(contact_w, MOVE_SPEED, force, eps, eps);
        } catch (const franka::Exception&) {
          // Even if something goes wrong, we consider ourselves "closed"
        }

        mode = GripMode::HOLD;
        g_grasp_pulse_until_ms.store(now_ms() + 1000, std::memory_order_relaxed);
      }

      // 3) OPEN sequence: HOLD -> OPEN
      if (mode == GripMode::HOLD &&
          lockout_ms == 0 &&
          open_ms >= OPEN_HOLD_MS) {
        open_ms    = 0;
        lockout_ms = LOCKOUT_AFTER_MS;

        try {
          gripper.move(W_MAX, MOVE_SPEED);  // fully open
        } catch (const franka::Exception&) {
          // ignore
        }

        mode = GripMode::OPEN;
      }

      // 4) Telemetry & feedback
      franka::GripperState gs = gripper.readOnce();
      g_grip_width = gs.width;

      bool grasp_like = (mode == GripMode::HOLD) && (gs.width < W_MAX - 5.0 * EPS_W);
      g_grasp_flag = grasp_like ? 1.0 : 0.0;

      if (grasp_like) {
        double fb = (g_grip_width / FB_REF) * FB_MAX;
        g_feedback = std::min(FB_MAX, std::max(0.0, fb));
      } else {
        g_feedback = 0.0;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_DT_MS));
    }

  } catch (const franka::Exception& e) {
    std::cerr << "[GRIP] " << e.what() << std::endl;
  }
}

// =================== MAIN ===================
static void on_sigint(int){ running.store(false); }

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname-or-ip>\n";
    return 1;
  }
  std::string robot_ip = argv[1];
  //std::signal(SIGINT, on_sigint);

  // Net threads
  std::thread t_rx(data_receiver_thread);
  std::thread t_poll(poll_reply_thread);
  std::thread t_grip(gripper_thread, robot_ip);

  try {
    franka::Robot robot(robot_ip);
    franka::Model model = robot.loadModel();
    setDefaultBehavior(robot);

    // === Rotate EE so tool-X aligns with gripper/J7 axis (old tool-Z) ===
    // Rotation: +90° about Y → R = [[0,0,1],[0,1,0],[-1,0,0]] (column-major)
    std::array<double, 16> T_F_EE = {
      0, 0, 1, 0,   // col 0
      0, 1, 0, 0,   // col 1
     -1, 0, 0, 0,   // col 2
      0, 0, 0, 1    // col 3 (no translation)
    };
    try {
      robot.setEE(T_F_EE);
      std::cout << "[EE] End effector frame rotated: tool-X ≡ gripper/J7 axis.\n";
    } catch (const franka::Exception& e) {
      std::cerr << "[EE] setEE failed: " << e.what()
                << "\nProceeding with default EE (tool-Z ≡ J7).\n";
      // If this fails, your wx bias will act on old tool-X; J7 won't be isolated.
    }

    // Move to start pose
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: robot will move. Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);

    robot.setCollisionBehavior(
  // joint torque lower / upper thresholds
  {{25,25,22,22,20,18,16}}, {{35,35,30,30,28,26,24}},
  {{25,25,22,22,20,18,16}}, {{35,35,30,30,28,26,24}},
  // Cartesian force lower / upper thresholds
  {{25,25,25,30,30,30}},    {{40,40,40,45,45,45}},
  {{25,25,25,30,30,30}},    {{40,40,40,45,45,45}}
);

    
    // === Continuous Cartesian velocity control ===
    while (running.load()) {
      robot.control([&](const franka::RobotState& s, franka::Duration period)
                    -> franka::CartesianVelocities {
        // --- time step (define once)
        // --- time step: MUST match libfranka's assumption (1 kHz)
        const double dt = franka::kDeltaT;  // 1e-3
        feedback2 = s.tau_J[6]; 
        

        // --- telemetry: collision flags (unchanged)
        bool any_contact = false;
        for (bool c : s.cartesian_contact)   if (c) any_contact = true;
        for (bool c : s.cartesian_collision) if (c) any_contact = true;
        g_collision_flag = any_contact ? 1.0 : 0.0;


        // --- tool-frame commands
        double wx_tool = 0.0, wy_tool = 0.0, wz_tool = 0.0;

        const int cls = (int)std::lround(g_class_id);
        switch (cls) {
          case 1: wy_tool = +W_FIXED_YZ; break; // extension
          case 2: wy_tool = -W_FIXED_YZ; break; // flexion
          case 3: wz_tool = +W_FIXED_YZ; break; // radial
          case 4: wz_tool = -W_FIXED_YZ; break; // ulnar
          default: break; // idle
        }


        // --- tool-frame linear velocity commands from PC (vx, vy, vz)
        double vx_tool = g_vx_tool;
        double vy_tool = g_vy_tool;
        double vz_tool = g_vz_tool;

        // --- PC connection watchdog: force zero motion if data is stale
        const uint64_t now   = now_ms();
        const uint64_t last  = g_last_data_ms.load(std::memory_order_relaxed);
        const bool data_stale = (last == 0) || (now - last > DATA_TIMEOUT_MS);

        if (data_stale) {
          // Force translations to zero
          vx_tool = 0.0;
          vy_tool = 0.0;
          vz_tool = 0.0;

          // Force Y/Z rotations to zero
          wy_tool = 0.0;
          wz_tool = 0.0;

          // wx_tool (tool-X / J7) still tracks phi_des, which is frozen at last command.
        }



        // --- rotation (columns = tool axes in base)
        const auto& T = s.O_T_EE;
        const double r00 = T[0*4 + 0], r01 = T[1*4 + 0], r02 = T[2*4 + 0];
        const double r10 = T[0*4 + 1], r11 = T[1*4 + 1], r12 = T[2*4 + 1];
        const double r20 = T[0*4 + 2], r21 = T[1*4 + 2], r22 = T[2*4 + 2];

        // --- Orientation delta: rotation vector φ (tool frame) relative to start ---
        static bool   have_R0 = false;
        static double R0[9];   // row-major neutral orientation
        double Rcur[9] = {
          r00, r01, r02,
          r10, r11, r12,
          r20, r21, r22
        };

        if (!have_R0) {
          // Store start orientation as neutral
          for (int i = 0; i < 9; ++i) R0[i] = Rcur[i];
          have_R0 = true;
        }

        // R_err = R0^T * Rcur  (row-major)
        double Rerr[9];
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
              sum += R0[k*3 + i] * Rcur[k*3 + j];  // (R0^T)_{ik} = R0_{ki}
            }
            Rerr[i*3 + j] = sum;
          }
        }

        // Rotation vector from R_err: φ = θ * u
        double rx = 0.0, ry = 0.0, rz = 0.0;
        double trace = Rerr[0] + Rerr[4] + Rerr[8];
        // Absolute orientation as ZYX Euler (yaw, pitch, roll)

        double cos_theta = (trace - 1.0) * 0.5;
        if (cos_theta > 1.0) cos_theta = 1.0;
        if (cos_theta < -1.0) cos_theta = -1.0;
        double theta = std::acos(cos_theta);

        if (theta > 1e-6) {
          double denom = 2.0 * std::sin(theta);
          double ux = (Rerr[7] - Rerr[5]) / denom; // (R32 - R23) / (2 sin θ)
          double uy = (Rerr[2] - Rerr[6]) / denom; // (R13 - R31)
          double uz = (Rerr[3] - Rerr[1]) / denom; // (R21 - R12)
          rx = theta * ux;
          ry = theta * uy;
          rz = theta * uz;
        }

        // Store tool-frame orientation deltas (relative to start)
        g_phi_x_tool = rx;
        g_phi_y_tool = ry;
        g_phi_z_tool = rz;


        // --- measured Cartesian twist from Jacobian * dq ---
        // J is 6x7, row-major: [vx; vy; vz; wx; wy; wz] in *base frame*
        std::array<double, 42> J = model.zeroJacobian(franka::Frame::kEndEffector, s);

        double vx_b = 0.0, vy_b = 0.0, vz_b = 0.0;
        double wx_b = 0.0, wy_b = 0.0, wz_b = 0.0;
        for (int j = 0; j < 7; ++j) {
          const double dqj = s.dq[j];

          vx_b += J[6 * j + 0] * dqj;  // row 0
          vy_b += J[6 * j + 1] * dqj;  // row 1
          vz_b += J[6 * j + 2] * dqj;  // row 2
          wx_b += J[6 * j + 3] * dqj;  // row 3
          wy_b += J[6 * j + 4] * dqj;  // row 4
          wz_b += J[6 * j + 5] * dqj;  // row 5
        }


        // Rotate base-frame twist into tool frame: v_tool = R^T * v_base, w_tool = R^T * w_base
        const double vx_tool_meas = r00 * vx_b + r10 * vy_b + r20 * vz_b;
        const double vy_tool_meas = r01 * vx_b + r11 * vy_b + r21 * vz_b;
        const double vz_tool_meas = r02 * vx_b + r12 * vy_b + r22 * vz_b;

        const double wx_tool_meas = r00 * wx_b + r10 * wy_b + r20 * wz_b;
        const double wy_tool_meas = r01 * wx_b + r11 * wy_b + r21 * wz_b;
        const double wz_tool_meas = r02 * wx_b + r12 * wy_b + r22 * wz_b;

        g_vx_tool_meas = vx_tool_meas;
        g_vy_tool_meas = vy_tool_meas;
        g_vz_tool_meas = vz_tool_meas;

        g_wx_tool_meas = wx_tool_meas;
        g_wy_tool_meas = wy_tool_meas;
        g_wz_tool_meas = wz_tool_meas;

                // --- Tool-X roll control in tool frame using φx = rx ---
        static bool   phi_init  = false;
        static double phi_des_f = 0.0;  // filtered desired φx [rad]

        // Actual roll about tool-X (relative to start)
        const double phi       = rx;                       // [rad]
        // Desired roll from glove (interpreted as φx in degrees around neutral)
        const double phi_des_raw = deg2rad(g_joint_rot_deg);

        // Initialise filtered setpoint at current pose
        if (!phi_init) {
          phi_des_f = phi;
          phi_init  = true;
        }

        // Limit how fast the desired angle can move
        const double V_DES_MAX = 0.5;   // rad/s (~30°/s)
        const double max_step  = V_DES_MAX * dt;

        double dphi_des = phi_des_raw - phi_des_f;
        if (dphi_des >  max_step) dphi_des =  max_step;
        if (dphi_des < -max_step) dphi_des = -max_step;
        phi_des_f += dphi_des;

        // PD on φx with velocity feedback from measured wx
        double e    = phi_des_f - phi;
        double dphi = -wx_tool_meas;    // [rad/s] roll rate in tool frame

        // Small deadband so it doesn't hunt for tiny errors
        const double E_DEADBAND = 5.0 * M_PI / 180.0;
        if (std::fabs(e) < E_DEADBAND) {
          e = 0.0;
        }

        const double KPX_local = 1.0;
        const double KDX_local = 0.05;

        wx_tool = KPX_local * e + KDX_local * dphi;

        // Clamp before accel limiter
        const double WX_PD_MAX = 0.5;   // rad/s
        if (wx_tool >  WX_PD_MAX) wx_tool =  WX_PD_MAX;
        if (wx_tool < -WX_PD_MAX) wx_tool = -WX_PD_MAX;


        // Angle: actual tool-frame rotation about X relative to start (φx)
        const double phi_tool = rx;                       // [rad], from rotation vector
        g_wx_tool_deg         = rad2deg(phi_tool);       // no extra 45° offset here


        // Torque: use external wrench at EE in base frame, project to tool-X.
        // RobotState::O_F_ext_hat_K = [Fx, Fy, Fz, Mx, My, Mz] in base frame.
        const auto& Fext        = s.O_F_ext_hat_K;
        const double Mx_base    = Fext[3];
        const double My_base    = Fext[4];
        const double Mz_base    = Fext[5];

        // tool-X axis expressed in base is the first column of R: (r00, r10, r20)
        // torque about tool-X = dot(M_base, e_x_tool) = R^T * M_base, first component
        const double Mx_tool    = r00 * Mx_base + r10 * My_base + r20 * Mz_base;

        g_tau_toolx             = Mx_tool;                // [Nm], torque about tool-X

        // --- External force: axial + radial in tool frame ---
        // Base-frame force components
        const double Fx_base = Fext[0];
        const double Fy_base = Fext[1];
        const double Fz_base = Fext[2];

        // Transform to tool frame: F_tool = R^T * F_base
        const double Fx_tool = r00 * Fx_base + r10 * Fy_base + r20 * Fz_base;
        const double Fy_tool = r01 * Fx_base + r11 * Fy_base + r21 * Fz_base;
        const double Fz_tool = r02 * Fx_base + r12 * Fy_base + r22 * Fz_base;

        // Axial force along peg (tool-X) and radial magnitude
        g_F_axial  = Fx_tool;  // [N]
        g_F_radial = std::sqrt(Fy_tool * Fy_tool + Fz_tool * Fz_tool);  // [N]

        // Position (last column of O_T_EE, column-major)
        const double px = T[12], py = T[13], pz = T[14];

        // Store pose for telemetry (meters)
        g_px = px;
        g_py = py;
        g_pz = pz;

        // Absolute orientation as ZYX Euler (yaw, pitch, roll)
        double yaw, pitch, roll;
        rot_to_euler_zyx(T, yaw, pitch, roll);

        // Store in roll–pitch–yaw form (radians)
        g_roll  = roll;
        g_pitch = pitch;
        g_yaw   = yaw;

        Prox px_p = classify_scalar(px, X_MIN, X_MAX, NEAR_D_POS);
        Prox py_p = classify_scalar(py, Y_MIN, Y_MAX, NEAR_D_POS);
        Prox pz_p = classify_scalar(pz, Z_MIN, Z_MAX, NEAR_D_POS);
  
        // Orientation borders using base-frame Euler ZYX
        Prox roll_p  = classify_scalar(roll,  ROLL_MIN,  ROLL_MAX,  NEAR_D_ANG);
        Prox pitch_p = classify_scalar(pitch, PITCH_MIN, PITCH_MAX, NEAR_D_ANG);
        Prox yaw_p   = classify_scalar(yaw,   YAW_MIN,   YAW_MAX,   NEAR_D_ANG);

        // Joint proximity vs limits
        Prox q_prox[7];
        for (size_t i = 0; i < 7; ++i) {
          q_prox[i] = classify_scalar(s.q[i], Q_MIN[i], Q_MAX[i], JOINT_NEAR);
        }

        // Base border state: position + *Euler* orientation
        int border_posori = border_state_from_prox({
            px_p, py_p, pz_p,
            roll_p, pitch_p, yaw_p
        });

        // Joint border state
        int border_joints = border_state_from_prox({q_prox[0], q_prox[1], q_prox[2],
                                                    q_prox[3], q_prox[4], q_prox[5],
                                                    q_prox[6]});

        // Combine: BORDER dominates, then NEAR, else FAR
        int border_state;
        if (border_posori == H_BORDER || border_joints == H_BORDER) {
          border_state = H_BORDER;
        } else if (border_posori == H_NEAR || border_joints == H_NEAR) {
          border_state = H_NEAR;
        } else {
          border_state = H_FAR;
        }

        g_border_state.store(border_state, std::memory_order_relaxed);
        // Joint-based slowdown factor: 1.0 in the middle, → 0 near limits
        double joint_scale = 1.0;
        for (size_t i = 0; i < 7; ++i) {
          double dmin = s.q[i] - Q_MIN[i];
          double dmax = Q_MAX[i] - s.q[i];
          double d    = std::min(dmin, dmax);   // distance to closer limit

          if (d <= 0.0) {
            // We are at / beyond a limit → no further motion
            joint_scale = 0.0;
          } else if (d < JOINT_NEAR) {
            // Inside “near” band: scale down linearly
            double alpha = d / JOINT_NEAR;      // alpha in (0,1)
            joint_scale = std::min(joint_scale, alpha);
          }
        }


        // Override with grasp pulse window (1 s) if active
        const uint64_t until = g_grasp_pulse_until_ms.load(std::memory_order_relaxed);
        const int haptic_state_out = (now_ms() < until) ? H_GRASP : border_state;

        // twist_cmd = [vx, vy, vz, wx, wy, wz] in tool frame
        static std::array<double, 6> twist_cmd{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        // ----- Contact-aware scaling (NEW) -----
        double contact_scale = 1.0;

        // Thresholds to tune:
        const double TAU_CONTACT = 1.5;  // [Nm] about tool-X
        const double SCALE_MIN   = 0.2;  // minimum scaling factor

        double tau_abs = std::fabs(Mx_tool);
        if (any_contact || tau_abs > TAU_CONTACT) {
          // Simple clamp: slam on the brakes in contact.
          contact_scale = SCALE_MIN;
        }

        // Also, you can smoothly reduce scale based on torque magnitude:
        // contact_scale = std::max(SCALE_MIN,
        //                          1.0 / (1.0 + 0.3 * std::max(0.0, tau_abs - TAU_CONTACT)));

        wx_tool *= contact_scale;
        wy_tool *= contact_scale;
        wz_tool *= contact_scale;
        vx_tool *= contact_scale;
        vy_tool *= contact_scale;
        vz_tool *= contact_scale;

        // 1) Build twist_target from commands
        std::array<double, 6> twist_target{
          vx_tool, vy_tool, vz_tool,
          wx_tool, wy_tool, wz_tool
        };

        // 1b) apply orientation gating and joint_scale to *twist_target* here
        twist_target[3] = soft_gate_angle(rx, twist_target[3], ROLL_MIN,  ROLL_MAX,  ANG_SOFT_ZONE);
        twist_target[4] = soft_gate_angle(ry, twist_target[4], PITCH_MIN, PITCH_MAX, ANG_SOFT_ZONE);
        twist_target[5] = soft_gate_angle(rz, twist_target[5], YAW_MIN,   YAW_MAX,   ANG_SOFT_ZONE);

        for (int i = 0; i < 6; ++i) twist_target[i] *= joint_scale;


        const double A_LIN   = A_MAX_CART;   // m/s^2
        const double A_ANG_YZ = ALPHA_MAX;   // rad/s^2 for Y/Z
        const double A_ANG_X  = ALPHA_MAX_X; // rad/s^2 for X

        // 2) Acceleration limiting (Δv / Δω per step)
        for (int i = 0; i < 6; ++i) {
          double a_cap;
          if (i < 3) {
            a_cap = A_LIN;           // vx, vy, vz
          } else if (i == 3) {
            a_cap = A_ANG_X;         // wx
          } else {
            a_cap = A_ANG_YZ;        // wy, wz
          }

          double dv     = twist_target[i] - twist_cmd[i];
          double max_dv = a_cap * dt;

          if (dv >  max_dv) dv =  max_dv;
          if (dv < -max_dv) dv = -max_dv;

          twist_cmd[i] += dv;
        }

        // 3) Speed limits
        
        // 3a) Linear: cap |v| at V_MAX_CART
        double vx_lin = twist_cmd[0];
        double vy_lin = twist_cmd[1];
        double vz_lin = twist_cmd[2];
        double v_norm = std::sqrt(vx_lin*vx_lin + vy_lin*vy_lin + vz_lin*vz_lin);

        if (v_norm > V_MAX_CART && v_norm > 1e-12) {
          double s = V_MAX_CART / v_norm;
          vx_lin *= s;
          vy_lin *= s;
          vz_lin *= s;
        }
        twist_cmd[0] = vx_lin;
        twist_cmd[1] = vy_lin;
        twist_cmd[2] = vz_lin;

        // 3b) Angular X: |wx| ≤ W_MAX_X
        if (std::fabs(twist_cmd[3]) > W_MAX_X) {
          twist_cmd[3] = std::copysign(W_MAX_X, twist_cmd[3]);
        }

        // 3c) Angular Y/Z: cap √(wy² + wz²) ≤ W_MAX_ANG
        double wy_ang = twist_cmd[4];
        double wz_ang = twist_cmd[5];
        double w_norm = std::sqrt(wy_ang*wy_ang + wz_ang*wz_ang);

        if (w_norm > W_MAX_ANG && w_norm > 1e-12) {
          double s = W_MAX_ANG / w_norm;
          wy_ang *= s;
          wz_ang *= s;
        }
        twist_cmd[4] = wy_ang;
        twist_cmd[5] = wz_ang;


        // 4) Overwrite tool-frame commands with smoothed ones
        vx_tool = twist_cmd[0];
        vy_tool = twist_cmd[1];
        vz_tool = twist_cmd[2];
        wx_tool = twist_cmd[3];
        wy_tool = twist_cmd[4];
        wz_tool = twist_cmd[5];

        // 5) Apply joint-limit slowdown (global scale in joint-near region)
        vx_tool *= joint_scale;
        vy_tool *= joint_scale;
        vz_tool *= joint_scale;
        wx_tool *= joint_scale;
        wy_tool *= joint_scale;
        wz_tool *= joint_scale;




        // --- tool → base map AFTER limiting
        // Angular
        double wx = r00*wx_tool + r01*wy_tool + r02*wz_tool;
        double wy = r10*wx_tool + r11*wy_tool + r12*wz_tool;
        double wz = r20*wx_tool + r21*wy_tool + r22*wz_tool;

        // Linear
        double vx = r00*vx_tool + r01*vy_tool + r02*vz_tool;
        double vy = r10*vx_tool + r11*vy_tool + r12*vz_tool;
        double vz = r20*vx_tool + r21*vy_tool + r22*vz_tool;

        // --- Hard positional borders in base frame (no orientation limiting yet) ---
        // If we're at or beyond a border, forbid velocity that would push further out.
        // Still allow velocity back toward the interior.
        // --- Smooth positional borders in base frame (no orientation limiting yet) ---
        vx = soft_gate_axis(px, vx, X_MIN, X_MAX, POS_SOFT_ZONE);
        vy = soft_gate_axis(py, vy, Y_MIN, Y_MAX, POS_SOFT_ZONE);
        vz = soft_gate_axis(pz, vz, Z_MIN, Z_MAX, POS_SOFT_ZONE);

        // --- tool-frame proportions for feedback (use limited tool-frame directly)
        auto clip = [](double v) { return std::max(-1.0, std::min(1.0, v)); };

        // Use *measured* ω_y / ω_max, ω_z / ω_max
        const double denom_w = (W_FIXED_YZ > 1e-9) ? W_FIXED_YZ : 1e-9;
        g_wy_prop = clip(g_wy_tool_meas / denom_w);
        g_wz_prop = clip(g_wz_tool_meas / denom_w);


       // --- Final joint-aware rate limiting with libfranka ---

        // Persistent history (static inside lambda = one instance per control loop)
        static std::array<double, 6> last_O_dP_EE_c{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        static std::array<double, 6> last_O_ddP_EE_c{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

        // Current commanded twist in base frame
        std::array<double, 6> O_dP_EE_c{{vx, vy, vz, wx, wy, wz}};

        // Call franka::limitRate on the twist arrays
        std::array<double, 6> limited_twist = franka::limitRate(
            franka::kMaxTranslationalVelocity,
            franka::kMaxTranslationalAcceleration,
            franka::kMaxTranslationalJerk,
            franka::kMaxRotationalVelocity,
            franka::kMaxRotationalAcceleration,
            franka::kMaxRotationalJerk,
            O_dP_EE_c,
            last_O_dP_EE_c,
            last_O_ddP_EE_c);

        // Update stored acceleration and velocity for next step
        for (size_t i = 0; i < 6; ++i) {
          last_O_ddP_EE_c[i] = (limited_twist[i] - last_O_dP_EE_c[i]) / dt;
          last_O_dP_EE_c[i]  = limited_twist[i];
        }

        // Wrap back into CartesianVelocities and return
        return franka::CartesianVelocities{limited_twist};



      });


      if (!running.load()) break;
    }

  } catch (const franka::Exception& e) {
    std::cerr << "[RT] Exception: " << e.what() << std::endl;
  }

  running.store(false);
  if (t_rx.joinable())   t_rx.join();
  if (t_poll.joinable()) t_poll.join();
  if (t_grip.joinable()) t_grip.join();

  std::cout << "Exiting.\n";
  return 0;
}
