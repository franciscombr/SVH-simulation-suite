

#include <GLFW/glfw3.h>
#include "mujoco/mujoco.h"
#include "ros/ros.h"

namespace viz {

void init();
void destroy();
void display();

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void scroll(GLFWwindow* window, double xoffset, double yoffset);

void init_local(mjModel* m);
void display_local(mjModel* m, mjData* d);

} // namespace viz

