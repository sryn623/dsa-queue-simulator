#ifndef WINDOW_H
#define WINDOW_H

#include <SDL3/SDL.h>
#include <string>

class Window {
public:
  Window(const std::string &title, int width, int height);
  ~Window();

  // To disble copy operator and assignment
  // this is done to ensure two instance of the Window is not create and make some bad things happen

  Window(const Window &) = delete;
  Window &operator=(const Window &) = delete;

  SDL_Window *get() const { return m_window; }
  SDL_Renderer *renderer() { return m_renderer; }
  void clear() const;
  void present() const;

private:
  SDL_Window *m_window = nullptr;
  SDL_Renderer *m_renderer = nullptr;
};

#endif