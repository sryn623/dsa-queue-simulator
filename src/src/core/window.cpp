#include "SDL3/SDL_error.h"
#include "SDL3/SDL_render.h"
#include "SDL3/SDL_video.h"
#include <stdexcept>
#include"window.h"


Window::Window(const std::string& title, int width, int height) {
  m_window = SDL_CreateWindow(title.c_str(), width, height, SDL_WINDOW_RESIZABLE);
  if (!m_window){
    throw  std::runtime_error(SDL_GetError());
  }

  m_renderer = SDL_CreateRenderer(m_window, nullptr);
  if (!m_renderer){
    SDL_DestroyWindow(m_window);
    throw  std::runtime_error(SDL_GetError());
  }

}

  Window::~Window() {
    SDL_DestroyWindow(m_window);
    SDL_DestroyRenderer(m_renderer);
  }


void Window::clear() const {


  SDL_SetRenderDrawColor(m_renderer, 0, 0, 0, 255);
  // Clear the current rendering target with the drawing color.

  SDL_RenderClear(m_renderer);
}

void Window::present() const {


  // Update the screen with any rendering performed since the previous call.
  SDL_RenderPresent(m_renderer);
}