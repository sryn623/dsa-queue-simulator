#include <iostream>
#include <SDL3/SDL.h>

int main(int argc, char* argv[]) {
    std::cout << "Starting SDL test program..." << std::endl;

    // Initialize SDL with verbose error reporting
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    std::cout << "SDL initialized successfully!" << std::endl;

    // Create a window
    SDL_Window* window = SDL_CreateWindow("SDL Test", 640, 480, 0);
    if (window == nullptr) {
        std::cout << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    std::cout << "Window created successfully!" << std::endl;

    // Create a renderer
    SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);
    if (renderer == nullptr) {
        std::cout << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    std::cout << "Renderer created successfully!" << std::endl;

    // Draw a simple shape
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);

    // Wait for 3 seconds
    std::cout << "Waiting for 3 seconds..." << std::endl;
    SDL_Delay(3000);

    // Cleanup
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    std::cout << "SDL shutdown successfully!" << std::endl;

    return 0;
}
