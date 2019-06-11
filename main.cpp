#include <vector>
#include <cmath>
#include <cstdlib>
#include <limits>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
Model *model = NULL;
const int width  = 800;
const int height = 800;

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

Vec3f barycentric(Vec2i *pts, Vec2i P) {
    Vec3f u = cross(Vec3f(pts[2].x-pts[0].x, pts[1].x-pts[0].x, pts[0].x-P.x), Vec3f(pts[2].y-pts[0].y, pts[1].y-pts[0].y, pts[0].y-P.y));
    /* `pts` and `P` has integer value as coordinates
       so `abs(u[2])` < 1 means `u[2]` is 0, that means
       triangle is degenerate, in this case return something with negative coordinates */
    if (std::abs(u.z)<1) return Vec3f(-1,1,1);
    return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z);
}

Vec3f barycentric(Vec3f A, Vec3f B, Vec3f C, Vec3f P) {
    Vec3f s[2];
    for (int i=2; i--; ) {
        s[i][0] = C[i]-A[i];
        s[i][1] = B[i]-A[i];
        s[i][2] = A[i]-P[i];
    }
    Vec3f u = cross(s[0], s[1]);
    if (std::abs(u[2])>1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z);
    return Vec3f(-1,1,1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color){
    //Checking for outlier triangles
    if(t0.y == t1.y && t0.y == t2.y)
        return;

    //Sorting points by height in order do divide triangle in 2 triangles with one horizontal side
    if (t0.y>t1.y) std::swap(t0, t1);
    if (t0.y>t2.y) std::swap(t0, t2);
    if (t1.y>t2.y) std::swap(t1, t2);

    int partTriangleHeight, wholeTriangleHeight = t2.y - t0.y;
    //Iterating through y values in the triangle like in scan conversion
    for(int y = 0; y < wholeTriangleHeight; y++){
        bool secongTriangle = y > t1.y - t0.y || t1.y ==  t0.y;
        partTriangleHeight = secongTriangle? t2.y - t1.y : t1.y - t0.y; //Checking which part triangle is being drawn ATM
        //Calculating scalars for determining limiting points in the current horizontal segment
        float p1 = (float)y / wholeTriangleHeight;
        float p2 = (float)(y - (secongTriangle? t1.y - t0.y : 0))/ partTriangleHeight;

        //Calculating limiting vectors (vertices) by getting the vector of the 2 barrieres (left and right)
        //multipying it by the scalars and then moving it in reference to t0 instead of the origin point
        Vec2i Left = t0 + (t2 - t0) * p1;
        Vec2i Right = secongTriangle? t1 + (t2 - t1) * p2 : t0 + (t1 - t0) * p2;
        if(Left.x > Right.x)
            std::swap(Left, Right);

        //Drawing one horizontal line segment between the limiting points "Left" and "Right"
        for(int x = Left.x; x <= Right.x; x++){
            image.set(x, y + t0.y, color);
        }
    }
}

void triangle_outline(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color) {
    line(t0, t1, image, color);
    line(t1, t2, image, color);
    line(t2, t0, image, color);
}

void triangle(Vec3f *pts, float *zbuffer, TGAImage &image, TGAColor color) {
    Vec2f bboxmin( std::numeric_limits<float>::max(),  std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    Vec2f clamp(image.get_width()-1, image.get_height()-1);
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            bboxmin[j] = std::max(0.f,      std::min(bboxmin[j], pts[i][j]));
            bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
        }
    }
    Vec3f P;
    for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) {
        for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) {
            Vec3f bc_screen  = barycentric(pts[0], pts[1], pts[2], P);
            if (bc_screen.x<0 || bc_screen.y<0 || bc_screen.z<0) continue;
            P.z = 0;
            for (int i=0; i<3; i++) P.z += pts[i][2]*bc_screen[i];
            if (zbuffer[int(P.x+P.y*width)]<P.z) {
                zbuffer[int(P.x+P.y*width)] = P.z;
                image.set(P.x, P.y, color);
            }
        }
    }
}

Vec3f world2screen(Vec3f v) {
    return Vec3f(int((v.x+1.)*width/2.+.5), int((v.y+1.)*height/2.+.5), v.z);
}

int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("object/african_head.obj");
    }

    float *zbuffer = new float[width*height];
    for (int i=width*height; i--; zbuffer[i] = -std::numeric_limits<float>::max());

    TGAImage image(width, height, TGAImage::RGB);
    Vec3f light_dir(0,0,-1);

    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec2i screen_coords[3];
        Vec3f world_coords[3];
        Vec3f pts[3];
        for (int j=0; j<3; j++) {
            Vec3f v = model->vert(face[j]);
            screen_coords[j] = Vec2i((v.x+1.)*width/2., (v.y+1.)*height/2.);
            world_coords[j]  = v;
            pts[j] = world2screen(v);
        }
        Vec3f n = cross((world_coords[2]-world_coords[0]), (world_coords[1]-world_coords[0]));
        n.normalize();
        float intensity = n*light_dir;
        if (intensity>0) {
            triangle(pts, zbuffer, image, TGAColor(intensity*255, intensity*255, intensity*255, 255));
            //triangle(screen_coords[0], screen_coords[1], screen_coords[2], image, TGAColor(intensity*255, intensity*255, intensity*255, 255));
        }
    }
    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}
