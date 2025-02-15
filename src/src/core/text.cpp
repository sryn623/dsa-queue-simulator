#include "text.h"
#include "SDL3_ttf/SDL_ttf.h"
#include <stdexcept>

Text::Text(SDL_Renderer *renderer, TTF_Font *font)
    : m_renderer(renderer), m_font(font), m_texture(nullptr),
      m_color{255, 255, 255, 255}, m_width(0), m_height(0) {
  if (!m_renderer || !m_font) {
    throw std::runtime_error("Invalid renderer or font");
  }
}

Text::~Text() { destroyTexture(); }

void Text::setText(const std::string& text, SDL_Color color){
    if (m_text == text &&
        m_color.r == color.r &&
        m_color.g == color.g &&
        m_color.b == color.b &&
        m_color.a == color.a) {
        return; // No change needed
    }

    m_text = text;
    m_color = color;
    createTexture();
}

void Text::render(int x, int y) const {
    if (!m_texture) return;

    SDL_FRect dstRect = {
        static_cast<float>(x),
        static_cast<float>(y),
        static_cast<float>(m_width),
        static_cast<float>(m_height)
    };

    SDL_RenderTextureRotated(m_renderer, m_texture, nullptr, &dstRect, 0.0, nullptr, SDL_FLIP_NONE);
}

void Text::createTexture() {
    destroyTexture();

    if (m_text.empty()) return;

    SDL_Surface* surface = TTF_RenderText_Blended(m_font, m_text.c_str(), m_text.size(), m_color);
    if (!surface) {
        throw std::runtime_error(SDL_GetError());
    }

    m_texture = SDL_CreateTextureFromSurface(m_renderer, surface);
    m_width = surface->w;
    m_height = surface->h;

    SDL_DestroySurface(surface);

    if (!m_texture) {
        throw std::runtime_error(SDL_GetError());
    }
}

void Text::destroyTexture() {
    if (m_texture) {
        SDL_DestroyTexture(m_texture);
        m_texture = nullptr;
    }
}