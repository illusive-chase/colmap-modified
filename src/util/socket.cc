#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "socket.h"

namespace mod {

  namespace {
    template <typename T>
    std::string Stringfy(T arg) {
      return std::to_string(arg);
    }

    template <>
    std::string Stringfy<std::string>(std::string arg) {
      return arg;
    }

    template <>
    std::string Stringfy<const char*>(const char* arg) {
      return arg;
    }

    template <typename ...T>
    std::string Concat(const T&... args) {
      return (Stringfy(args) + ...);
    }

    template <typename ...T>
    bool Parse(const std::string& source, T&... args) {
      std::istringstream iss { source };
      (iss >> ... >> args);
      return !iss.fail();
    }
  }

  Socket::Socket(const std::string& address, int port): address(address), port(port) {
    sock = ::socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr;
    ::memset(&addr, 0, sizeof(addr));
    addr.sin_port = htons(port);
    addr.sin_family = AF_INET;
    ::inet_pton(AF_INET, address.c_str(), &addr.sin_addr);
    if (::connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      ::perror("connect failed");
      close();
    }
  }

  Socket::~Socket() {
    close();
  }

  bool Socket::connected() const {
    return sock != -1;
  }


  bool Socket::send(const std::string& data) {
    if (::send(sock, data.c_str(), data.length() + 1, 0) < 0) {
      ::perror("send failed");
      return false;
    }
    return true;
  }

  std::string Socket::recv() {
    char buffer[512] = {};
    std::string reply;
    while (true) {
      ssize_t len = ::recv(sock, buffer, sizeof(buffer) - 1, 0);
      if (len == 0) {
        close();
        break;
      }
      if (len < 0) {
        ::perror("recv failed");
        break;
      }
      buffer[len] = 0;
      reply += buffer;
      if (buffer[len - 1] == 0) break; // protocol
    }
    return reply;
  }

  void Socket::close() {
    if (sock != -1) {
      ::close(sock);
      sock = -1;
    }
  }


  TCPClient::TCPClient(const std::string& address, int port, bool normal_log, bool failure_log): socket(address, port), normal_log(normal_log), failure_log(failure_log) {
    if (socket.connected()) {
      socket.send("connected");
      Validate("acknowledge");
    }
  }


  bool TCPClient::Connected() const {
    return socket.connected();
  }

  void TCPClient::Validate(const char* expected) {
    if (socket.recv() != expected) socket.close();
  }


  void TCPClient::BeginReconstruction(int num_init_trials) {
    if (!normal_log) return;
    socket.send(Concat(
      "[Begin Reconstruction]\n",
      "    This is ", num_init_trials + 1, "th initial trial.\n"
    ));
    Validate("acknowledge");
  }

  bool TCPClient::Abort() {
    if (!normal_log) return false;
    socket.send(Concat(
      "[Abort]\n",
      "    Do you want to abort this reconstruction? y/n\n"
    ));
    std::string resp = socket.recv();
    if (resp != "y" && resp != "yes") {
      socket.send("    Continue...\n");
      Validate("acknowledge");
      return false;
    }
    socket.send("    Abort.\n");
    Validate("acknowledge");
    return true;
  }

  void TCPClient::FindInitialImagePair(uint32_t* img1, uint32_t* img2) {
    if (!normal_log) return;
    if (*img1 == ~0U || *img2 == ~0U) {
      socket.send(Concat(
        "[Find Initial Image Pair]\n",
        "    Do you want to mannually specify? ${img1} ${img2}.\n",
        "    Anything else for not.\n"
      ));
    } else {
      socket.send(Concat(
        "[Find Initial Image Pair]\n",
        "    To use pair #", *img1, ", #", *img2, " from options.\n",
        "    Do you want to mannually specify? ${img1} ${img2}.\n",
        "    Anything else for not.\n"
      ));
    }
    std::string resp = socket.recv();
    uint32_t a, b;
    if (Parse(resp, a, b)) {
      *img1 = a;
      *img2 = b;
      socket.send(Concat("    Mannually specify #", a, ", #", b, ".\n"));
    } else {
      socket.send("    Skip.\n");
    }
    Validate("acknowledge");
  }

  void TCPClient::InitializeWithImagePair(uint32_t img1, uint32_t img2) {
    if (!normal_log) return;
    socket.send(Concat(
      "[Initialize With ImagePair]\n",
      "    To use pair #", img1, ", #", img2, ".\n"
    ));
    Validate("acknowledge");
  }

  void TCPClient::FailInitialization(uint32_t img1, uint32_t img2, double min_tri_angle, int min_num_inliers) {
    if (!normal_log) return;
    socket.send(Concat(
      "[Fail Initialization]\n",
      "    Used pair #", img1, ", #", img2, ".\n",
      "    Used min_tri_angle = ", min_tri_angle, ".\n",
      "    Used min_num_inliers = ", min_num_inliers, ".\n"
    ));
    Validate("acknowledge");
  }

  bool TCPClient::RelaxAndRestart(int* min_num_inliers, double* min_tri_angle) {
    if (!normal_log) return false;
    socket.send(Concat(
      "[Relax And Restart]\n",
      "    To use min_tri_angle = ", *min_tri_angle, ".\n",
      "    To use min_num_inliers = ", *min_num_inliers, ".\n"
      "    Do you want to mannually specify? ${min_tri_angle} ${min_num_inliers}.\n",
      "    Or 'q' or 'quit' to give up.\n",
      "    Anything else for not to specify but to continue.\n"
    ));
    std::string resp = socket.recv();
    int a; double b;
    if (resp == "q" || resp == "quit") {
      socket.send("    Quit.\n");
      Validate("acknowledge");
      return false;
    }
    if (Parse(resp, b, a)) {
      *min_num_inliers = a;
      *min_tri_angle = b;
      socket.send(Concat(
        "    Mannually specify:\n",
        "    min_tri_angle = ", b, ".\n",
        "    min_num_inliers = ", a, ".\n"
      ));
    } else {
      socket.send("    Skip.\n");
    }
    Validate("acknowledge");
    return true;
  }

  void TCPClient::SucceedInitialRegistration(uint32_t img1, uint32_t img2, int num_reg_images, int num_points_3d) {
    if (!normal_log) return;
    socket.send(Concat(
      "[Succeed Initial Registration]{", img1, ",", img2, "}\n",
      "    Used pair #", img1, ", #", img2, ".\n",
      "    Registered images: ", num_reg_images, ".\n",
      "    Fused 3d points: ", num_points_3d, ".\n"
    ));
    Validate("acknowledge");
  }

  void TCPClient::FailInitialRegistration(uint32_t img1, uint32_t img2, int num_reg_images, int num_points_3d) {
    if (!normal_log) return;
    socket.send(Concat(
      "[Fail Initial Registration]{", img1, ",", img2, "}\n",
      "    Used pair #", img1, ", #", img2, ".\n",
      "    Registered images: ", num_reg_images, ".\n",
      "    Fused 3d points: ", num_points_3d, ".\n"
    ));
    Validate("acknowledge");
  }

  void TCPClient::FindNextImages(const std::vector<uint32_t>& next_images) {
    if (!normal_log) return;
    std::string ids = "";
    for (uint32_t id: next_images) ids = Concat(ids, id, ",");
    if (ids.length()) ids.pop_back();
    socket.send(Concat(
      "[Find Next Images]{", ids, "}\n",
      "    Next images to use: [", ids, "].\n"
    ));
    Validate("acknowledge");
  }

  void TCPClient::RegisterNextImage(uint32_t* next_image_id, int num_reg_images) {
    if (!normal_log) return;
    socket.send(Concat(
      "[Register Next Image]\n",
      "    ", num_reg_images, " images are already registered.\n",
      "    The next image to register is #", *next_image_id, ".\n",
      "    Do you want to mannually specify? ${next_image_id}.\n",
      "    Anything else for not.\n"
    ));
    std::string resp = socket.recv();
    uint32_t a;
    if (Parse(resp, a)) {
      *next_image_id = a;
      socket.send(Concat("    Mannually specify #", a, ".\n"));
    } else {
      socket.send("    Skip.\n");
    }
    Validate("acknowledge");
  }

  void TCPClient::SucceedRegistration(uint32_t image_id, int num_reg_images, int num_points_3d) {
    if (!normal_log) return;
    socket.send(Concat(
      "[Succeed Registration]{", image_id, "}\n",
      "    ", num_reg_images, " images are already registered.\n",
      "    ", num_points_3d, " 3d points are already fused.\n"
    ));
    Validate("acknowledge");
  }

  void TCPClient::FailRegistration(uint32_t image_id, int num_reg_trials, int num_reg_images) {
    if (!normal_log) return;
    socket.send(Concat(
      "[Fail Registration]{", image_id, "}\n",
      "    This is ", num_reg_trials + 1, "th register trial.\n",
      "    ", num_reg_images, " images are already registered.\n"
    ));
    Validate("acknowledge");
  }

  bool TCPClient::GiveUp() {
    if (!normal_log) return false;
    socket.send(Concat(
      "[Give Up]\n",
      "    Do you want to give up registering rest images? y/n\n",
      "    Anything else for not.\n"
    ));
    std::string resp = socket.recv();
    if (resp != "y" && resp != "yes") {
      socket.send("    Continue...\n");
      Validate("acknowledge");
      return false;
    }
    socket.send("    Quit.\n");
    Validate("acknowledge");
    return true;
  }

  void TCPClient::FailAllRegistration(bool* reg_next_success, bool* prev_reg_next_success) {
    if (!normal_log) return;
    if (*reg_next_success) return;
    if (*prev_reg_next_success) {
      socket.send(Concat(
        "[Fail All Registration]\n",
        "    None of images can be registered.\n",
        "    It will retry after an Iterative Global Refinement.\n"
      ));
      Validate("acknowledge");
      return;
    }
    socket.send(Concat(
      "[Fail All Registration]\n",
      "    None of images can be registered.\n",
      "    It has already retried after an Iterative Global Refinement.\n",
      "    Do you want to force retrying? y/n\n"
    ));
    
    std::string resp = socket.recv();
    if (resp != "y" && resp != "yes") {
      socket.send("    Skip.\n");
    } else {
      *prev_reg_next_success = true;
      socket.send("    Force retrying.\n");
    }
    Validate("acknowledge");
  }

  void TCPClient::EndReconstruction(int num_init_trials, int num_reg_images, int num_points_3d) {
    if (!normal_log) return;
    socket.send(Concat(
      "[End Reconstruction]\n",
      "    This is ", num_init_trials + 1, "th initial trial.\n",
      "    ", num_reg_images, " images are registered.\n"
      "    ", num_points_3d, " 3d points are fused.\n"
    ));
    Validate("acknowledge");
  }

  void TCPClient::FailDueToBadOverlap(uint32_t img1, uint32_t img2) {
    if (!failure_log) return;
    socket.send(Concat(
      "[Fail Due To Bad Overlap]{", img1, ",", img2, "}\n"
    ));
    Validate("acknowledge");
  }


  void TCPClient::FailDueToLittleTriAngle(uint32_t img1, uint32_t img2) {
    if (!failure_log) return;
    socket.send(Concat(
      "[Fail Due To Little Tri Angle]{", img1, ",", img2, "}\n"
    ));
    Validate("acknowledge");
  }

  void TCPClient::FailDueToLittleVisible3DPoints(uint32_t img, const std::vector<double>& xys) {
    if (!failure_log) return;
    std::string string_xys;
    for (double v : xys) {
      string_xys += ",";
      string_xys += std::to_string(static_cast<int>(v));
    }
    socket.send(Concat(
      "[Fail Due To Little Visible 3D Points]{", img, string_xys, "}\n"
    ));
    Validate("acknowledge");
  }

  void TCPClient::FailDueToLittleTri2DPoints(uint32_t img, const std::vector<double>& xys) {
    if (!failure_log) return;
    std::string string_xys;
    for (double v : xys) {
      string_xys += ",";
      string_xys += std::to_string(static_cast<int>(v));
    }
    socket.send(Concat(
      "[Fail Due To Little Tri 2D Points]{", img, string_xys, "}\n"
    ));
    Validate("acknowledge");
  }

  void TCPClient::FailDueToBadPoseEstimation(uint32_t img) {
    if (!failure_log) return;
    socket.send(Concat(
      "[Fail Due To Bad Pose Estimation]{", img, "}\n"
    ));
    Validate("acknowledge");
  }

  void TCPClient::FailDueToLittle2DInliers(uint32_t img, const std::vector<double>& xys) {
    if (!failure_log) return;
    std::string string_xys;
    for (double v : xys) {
      string_xys += ",";
      string_xys += std::to_string(static_cast<int>(v));
    }
    socket.send(Concat(
      "[Fail Due To Little 2D Inliers]{", img, string_xys, "}\n"
    ));
    Validate("acknowledge");
  }

  void TCPClient::FailDueToBadPoseRefinement(uint32_t img) {
    if (!failure_log) return;
    socket.send(Concat(
      "[Fail Due To Bad Pose Refinement]{", img, "}\n"
    ));
    Validate("acknowledge");
  }


} // namespace mod
