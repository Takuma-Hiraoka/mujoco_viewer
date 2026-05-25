#pragma once

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <string>
#include <memory>

namespace mujoco_viewer {
  class Viewer {
  public:
    void viewModel(std::string filename);
    void update(mjData& data_);
    void setData(const mjData& data);
    void updateScene();
    mjvGeom* appendGeom();
    void drawScene();
    void pollEvents();
    void render();
    bool isOpen() const;

    mjModel* model() const { return model_.get(); }
    mjData* data() { return data_.get(); }
    const mjData* data() const { return data_.get(); }
    mjvScene* scene() { return &scn; }
    const mjvScene* scene() const { return &scn; }
    mjvCamera* camera() { return &cam; }
    mjvOption* option() { return &opt; }
    void view();
  private:
    static void mouse_button_callback(GLFWwindow* window, int button, int act, int mods);
    void mouse_button(GLFWwindow* window, int button, int act, int mods);
    static void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);
    void mouse_move(GLFWwindow* window, double xpos, double ypos);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
    void scroll(GLFWwindow* window, double xoffset, double yoffset);
    std::shared_ptr<mjModel> model_ = nullptr;
    std::shared_ptr<mjData> data_ = nullptr;

    double lastx, lasty;
    bool button_left = false;
    bool button_right = false;
    GLFWwindow* window;
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;

  };
}
