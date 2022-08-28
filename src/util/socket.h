#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

namespace mod {

class Socket {
private:
  int sock;
  std::string address;
  int port;

public:
  Socket(const std::string& address, int port);
  ~Socket();
  Socket(const Socket&) = delete;
  
  bool connected() const;
  bool send(const std::string& data);
  std::string recv();
  void close();
};

class TCPClient {
private:
  Socket socket;
  void Validate(const char* expected);

public:
  TCPClient(const std::string& address, int port);
  TCPClient(const TCPClient&) = delete;
  bool Connected() const;

  void BeginReconstruction(int num_init_trials);
  bool Abort();
  void FindInitialImagePair(uint32_t* img1, uint32_t* img2);
  void InitializeWithImagePair(uint32_t img1, uint32_t img2);
  void FailInitialization(uint32_t img1, uint32_t img2, double min_tri_angle, int min_num_inliers);
  bool RelaxAndRestart(int* min_num_inliers, double* min_tri_angle);
  void SucceedInitialRegistration(uint32_t img1, uint32_t img2, int num_reg_images, int num_points_3d);
  void FailInitialRegistration(uint32_t img1, uint32_t img2, int num_reg_images, int num_points_3d);
  void FindNextImages(const std::vector<uint32_t>& next_images);
  void RegisterNextImage(uint32_t* next_image_id, int num_reg_images);
  void SucceedRegistration(uint32_t image_id, int num_reg_images, int num_points_3d);
  void FailRegistration(uint32_t image_id, int num_reg_trials, int num_reg_images);
  bool GiveUp();
  void FailAllRegistration(bool* reg_next_success, bool* prev_reg_next_success);
  void EndReconstruction(int num_init_trials, int num_reg_images, int num_points_3d);
};

} // namespace mod

#endif