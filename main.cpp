#include <vector>
#include <cmath>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
Model *model = NULL;
const int width  = 800;
const int height = 800;

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
    bool steep = false;
    if (std::abs(x0-x1)<std::abs(y0-y1)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0>x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    for (int x=x0; x<=x1; x++) {
        float t = (x-x0)/(float)(x1-x0);
        int y = y0*(1.-t) + y1*t;
        if (steep) {
            image.set(y, x, color);
        } else {
            image.set(x, y, color);
        }
    }
}

void line(Vec2i v0, Vec2i v1, TGAImage &image, TGAColor color) {
    bool steep = false;
    if (std::abs(v0.x-v1.x)<std::abs(v0.y - v1.y)) {
        std::swap(v0.x, v0.y);
        std::swap(v1.x, v1.y);
        steep = true;
    }
    if (v0.x > v1.x) {
        std::swap(v0.x, v1.x);
        std::swap(v0.y, v1.y);
    }

    for (int x=v0.x; x<=v1.x; x++) {
        float t = (x-v0.x)/(float)(v1.x-v0.x);
        int y = v0.y*(1.-t) + v1.y*t;
        if (steep) {
            image.set(y, x, color);
        } else {
            image.set(x, y, color);
        }
    }
}

void triangle(Vec2i v0, Vec2i v1, Vec2i v2, TGAImage &image, TGAColor color){
    line(v0, v1, image, color);
    line(v0, v2, image, color);
    line(v1,v2, image, color);
}

int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("object/african_head.obj");
    }

    TGAImage image(width, height, TGAImage::RGB);

    triangle(Vec2i(10, 70), Vec2i(50, 160), Vec2i(70, 80), image, red);

    image.flip_vertically();
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}
