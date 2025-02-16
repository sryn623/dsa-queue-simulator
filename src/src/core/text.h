

#ifndef TEXT_H
#define TEXT_H

#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <string>

class Text {
public:
  Text(SDL_Renderer *renderer, TTF_Font *font);
  ~Text();

  Text(const Text &) = delete;
  Text &operator=(const Text) = delete;

  void setText(const std::string &text, SDL_Color color = {255, 255, 255, 255});
  void render(int x, int y) const;
  [[nodiscard]] int getWidth() const { return m_width; }
  [[nodiscard]] int getHeight() const { return m_height; }

private:
  SDL_Renderer *m_renderer;
  TTF_Font *m_font;
  SDL_Texture *m_texture;
  std::string m_text;
  SDL_Color m_color;
  int m_width;
  int m_height;

  void createTexture();
  void destroyTexture();
};

#endif // TEXT_H