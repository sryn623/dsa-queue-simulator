#ifndef MAIN_H
#define MAIN_H

#include "window.h"
#include <SDL3/SDL.h>
#include <string>

class App {
public:
  App();
  ~App();

  void run();

private:
  void update();
  void render();
  void process_event();

  Window m_window;
  bool m_running = true;
};

#endif // !MAIN_H
