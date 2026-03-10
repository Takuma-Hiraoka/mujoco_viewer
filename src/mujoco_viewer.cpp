#include <mujoco_viewer/mujoco_viewer.h>
#include <iostream>

namespace mujoco_viewer {
  void Viewer::mouse_button_callback(GLFWwindow* window, int button, int act, int mods)
  {
    Viewer* self = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    self->mouse_button(window, button, act, mods);
  }
  void Viewer::mouse_button(GLFWwindow* window, int button, int act, int mods)
  {
    if(button==GLFW_MOUSE_BUTTON_LEFT)
      button_left = (act==GLFW_PRESS);
    if(button==GLFW_MOUSE_BUTTON_RIGHT)
      button_right = (act==GLFW_PRESS);

    glfwGetCursorPos(window, &this->lastx, &this->lasty);
  }

  void Viewer::mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
  {
    Viewer* self = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    self->mouse_move(window, xpos, ypos);
  }
  void Viewer::mouse_move(GLFWwindow* window, double xpos, double ypos)
  {
    if(!this->button_left && !this->button_right) return;

    double dx = xpos-this->lastx;
    double dy = ypos-this->lasty;
    this->lastx = xpos;
    this->lasty = ypos;

    int w,h;
    glfwGetWindowSize(window,&w,&h);

    mjtMouse action;
    if(this->button_left)
      action = mjMOUSE_ROTATE_V;
    else
      action = mjMOUSE_MOVE_H;

    mjv_moveCamera(this->model_.get(), action, dx/w, dy/h, &this->scn, &this->cam);
  }

  void Viewer::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
  {
    Viewer* self = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    self->scroll(window, xoffset, yoffset);
  }
  void Viewer::scroll(GLFWwindow* window, double xoffset, double yoffset)
  {
    mjv_moveCamera(this->model_.get(), mjMOUSE_ZOOM, 0, -0.05*yoffset, &this->scn, &this->cam);
  }

  void Viewer::viewModel(std::string filename) {
    this->model_ = std::shared_ptr<mjModel>(mj_loadXML(filename.c_str(), nullptr, nullptr, 0), mj_deleteModel);
    if (!model_) std::cerr << "cannot load model" << std::endl;
    this->data_ = std::shared_ptr<mjData>(mj_makeData(this->model_.get()), mj_deleteData);
    glfwInit();
    this->window = glfwCreateWindow(1200, 900, "viewer", NULL, NULL);
    glfwMakeContextCurrent(this->window);
    glfwSetWindowUserPointer(this->window, this);
    glfwSetMouseButtonCallback(this->window, mouse_button_callback);
    glfwSetCursorPosCallback(this->window, mouse_move_callback);
    glfwSetScrollCallback(this->window, scroll_callback);

    mjv_defaultCamera(&this->cam);
    mjv_defaultOption(&this->opt);
    mjv_defaultScene(&this->scn);
    mjr_defaultContext(&this->con);

    mjv_makeScene(this->model_.get(), &this->scn, 2000);
    mjr_makeContext(this->model_.get(), &this->con, mjFONTSCALE_150);
    // mjr_freeContext(&this->con);
    // mjv_freeScene(&this->scn);
    // glfwTerminate();
  }

  void Viewer::update(mjData& data_) {
    *this->data_ = data_;
    mj_forward(this->model_.get(), this->data_.get());
    mjv_updateScene(this->model_.get(), this->data_.get(), &this->opt, NULL, &this->cam, mjCAT_ALL, &this->scn);
    int w,h;
    glfwGetFramebufferSize(this->window,&w,&h);
    mjrRect viewport = {0,0,w,h};
    mjr_render(viewport, &this->scn, &this->con);
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  void Viewer::view() {
    while (!glfwWindowShouldClose(this->window))
      {
        mjv_updateScene(this->model_.get(), this->data_.get(), &this->opt, NULL, &this->cam, mjCAT_ALL, &this->scn);

        int w,h;
        glfwGetFramebufferSize(window,&w,&h);
        mjrRect viewport = {0,0,w,h};

        mjr_render(viewport, &this->scn, &this->con);

        glfwSwapBuffers(window);
        glfwPollEvents();
      }
  }
}
