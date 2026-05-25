#include <mujoco_viewer/mujoco_viewer.h>
#include <iostream>
#include <algorithm>

namespace mujoco_viewer {
  namespace {
    void fitCameraToModel(const mjModel* model, const mjData* data, mjvCamera& cam)
    {
      bool found = false;
      mjtNum minpos[3] = {0, 0, 0};
      mjtNum maxpos[3] = {0, 0, 0};
      for (int i = 0; i < model->ngeom; ++i) {
        if (model->geom_type[i] == mjGEOM_PLANE) continue;
        const mjtNum* pos = data->geom_xpos + 3 * i;
        if (!found) {
          for (int j = 0; j < 3; ++j) {
            minpos[j] = pos[j];
            maxpos[j] = pos[j];
          }
          found = true;
        } else {
          for (int j = 0; j < 3; ++j) {
            minpos[j] = std::min(minpos[j], pos[j]);
            maxpos[j] = std::max(maxpos[j], pos[j]);
          }
        }
      }

      if (!found) return;
      mjtNum span = 0;
      for (int j = 0; j < 3; ++j) {
        cam.lookat[j] = 0.5 * (minpos[j] + maxpos[j]);
        span = std::max(span, maxpos[j] - minpos[j]);
      }
      cam.distance = std::max<mjtNum>(2.5 * span, model->stat.extent);
      cam.azimuth = model->vis.global.azimuth;
      cam.elevation = model->vis.global.elevation;
    }
  }

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
    char error[1024] = {0};
    this->model_ = std::shared_ptr<mjModel>(mj_loadXML(filename.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
    if (!model_) {
      std::cerr << "cannot load model: " << filename << std::endl;
      std::cerr << error << std::endl;
      return;
    }
    this->data_ = std::shared_ptr<mjData>(mj_makeData(this->model_.get()), mj_deleteData);
    mj_resetData(this->model_.get(), this->data_.get());
    mj_forward(this->model_.get(), this->data_.get());
    glfwInit();
    this->window = glfwCreateWindow(1200, 900, "viewer", NULL, NULL);
    if (!this->window) {
      std::cerr << "cannot create GLFW window" << std::endl;
      return;
    }
    glfwMakeContextCurrent(this->window);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(this->window, this);
    glfwSetMouseButtonCallback(this->window, mouse_button_callback);
    glfwSetCursorPosCallback(this->window, mouse_move_callback);
    glfwSetScrollCallback(this->window, scroll_callback);

    mjv_defaultCamera(&this->cam);
    mjv_defaultOption(&this->opt);
    mjv_defaultScene(&this->scn);
    mjr_defaultContext(&this->con);
    mjv_defaultFreeCamera(this->model_.get(), &this->cam);
    fitCameraToModel(this->model_.get(), this->data_.get(), this->cam);
    for (int i = 0; i < mjNGROUP; ++i) {
      this->opt.geomgroup[i] = 1;
    }

    mjv_makeScene(this->model_.get(), &this->scn, 2000);
    mjr_makeContext(this->model_.get(), &this->con, mjFONTSCALE_150);
    // mjr_freeContext(&this->con);
    // mjv_freeScene(&this->scn);
    // glfwTerminate();
  }

  void Viewer::update(mjData& data_) {
    this->setData(data_);
    this->render();
  }

  void Viewer::setData(const mjData& data_) {
    mj_copyData(this->data_.get(), this->model_.get(), &data_);
    mj_forward(this->model_.get(), this->data_.get());
  }

  void Viewer::updateScene() {
    if (!this->model_ || !this->data_ || !this->window) return;
    mjv_updateScene(this->model_.get(), this->data_.get(), &this->opt, NULL, &this->cam, mjCAT_ALL, &this->scn);
  }

  mjvGeom* Viewer::appendGeom() {
    if (this->scn.ngeom >= this->scn.maxgeom) return nullptr;
    return &this->scn.geoms[this->scn.ngeom++];
  }

  void Viewer::drawScene() {
    if (!this->window) return;
    int w,h;
    glfwGetFramebufferSize(this->window,&w,&h);
    mjrRect viewport = {0,0,w,h};
    mjr_render(viewport, &this->scn, &this->con);
    glfwSwapBuffers(window);
  }

  void Viewer::pollEvents() {
    glfwPollEvents();
  }

  void Viewer::render() {
    this->updateScene();
    // geomを追加したい場合はupdateScene, drawScene, pollEventsを自分の関数内で行う
    // updateSceneとdrawSceneの間でmjv_initGeomを呼ぶこと
    // 
    // if (mjvGeom* geom = viewer.appendGeom()) {
    //   mjv_initGeom(geom, mjGEOM_SPHERE, size, pos, nullptr, rgba);
    // }
    this->drawScene();
    this->pollEvents();
  }

  bool Viewer::isOpen() const {
    if (!this->window) return false;
    return !glfwWindowShouldClose(this->window);
  }

  void Viewer::view() {
    while (this->isOpen())
      {
        this->render();
      }
  }
}
