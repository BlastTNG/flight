/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "texture.h"
#if defined(_MSC_VER)
    #define NOMINMAX
    #include <windows.h>
#endif
#include <cstdlib>
#include "math.h"
#include <tiffio.h>
#include "GL/gl.h"
#include "glhelper.h"
#include "../logger_main.h"

using namespace Displaying;
using Main::logger;
using std::min;
using std::max;


Texture::Texture()
{
    size = 0;
    used_width = 1.0;
    used_height = 1.0;
}

void Texture::init(int new_size, bool display_enabled)
{
    if (display_enabled) {
		glGenTextures(1, &id);
        size_to_at_least(new_size);
    }
}

void Texture::init(bool display_enabled)
{
    init(64, display_enabled);
}

void Texture::destroy(bool display_enabled)
{
    if (display_enabled) {
        glDeleteTextures(1, &id);
    }
}

Texture::~Texture()
{
}

void Texture::size_to_at_least(int min_size)
{
    if (size < min_size) {
        int max_exp = 13;	//changed from 13
        int exp = 6;
        while ((exp < max_exp) && (int(pow(2.0, exp)) < min_size)) exp++;
        int new_size = int(pow(2.0, exp));
        if (bind_blank(new_size, false)) {
            size = new_size;
        }
    }
}

int Texture::bind_blank(int new_size, bool black)
{
    char* data;
	int maxval;
    data = (char*) malloc(sizeof(char)*new_size*new_size*4);
    for(int j=0; j<new_size; j++){
        for(int i=0; i<new_size; i++){
            data[(j*new_size+i)*4+0] = 0;
            data[(j*new_size+i)*4+1] = 0;
            data[(j*new_size+i)*4+2] = 0;
            if (black) {
                data[(j*new_size+i)*4+3] = char(255);
            } else {
                data[(j*new_size+i)*4+3] = 0; // why?
            }
        }
    }
    glBindTexture(GL_TEXTURE_2D, id); // Bind 0
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxval);		//added by KNS
		if (new_size <= maxval) {						//added by KNS
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, new_size, new_size,
        //0, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, data);
        0, GL_RGBA, GL_UNSIGNED_BYTE, data);
		}
	free(data);
    if (!glGetError()) return 1;
    return 0;
}

void Texture::bind_data(char* data, int width, int height)
{
   	size_to_at_least(max(width, height));
	glBindTexture(GL_TEXTURE_2D, id); // Bind 1
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height,
        //GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, data);
        GL_RGBA, GL_UNSIGNED_BYTE, data);
	//if (!glGetError()) {
	GLenum error;
		error = glGetError();
	if (error == 0) {							//added by KNS to check errors
        used_width = double(width) / double(size);
        used_height = double(height) / double(size);
    }
	logger.log(format("GL failed: %s") % error);
    glBindTexture(GL_TEXTURE_2D, 0); // unBind 1
}

void Texture::load_tiff(std::string filename)
{
    TIFF* image = TIFFOpen(filename.c_str(), "r");
    if (image) {
        uint32 width = 0;
        uint32 height = 0;
        size_t num_pixels;
        uint32* raster;

        TIFFGetField(image, TIFFTAG_IMAGEWIDTH, &width);
        TIFFGetField(image, TIFFTAG_IMAGELENGTH, &height);
        num_pixels = width * height;
        raster = (uint32*) _TIFFmalloc(num_pixels * sizeof(uint32));
        if (raster != NULL) {
            if (TIFFReadRGBAImage(image, width, height, raster, 0)) {
                glBindTexture(GL_TEXTURE_2D, id); // Bind 2
                size_to_at_least(max(width, height));
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height,
                    GL_RGBA, GL_UNSIGNED_BYTE, raster);
                if (!glGetError()) {
                    used_width = double(width) / double(size);
                    used_height = double(height) / double(size);
                }
                glBindTexture(GL_TEXTURE_2D, 0); // unBind 2
            }
            _TIFFfree(raster);
        }
        TIFFClose(image);
    }
    else {
        logger.log(format("Warning: could not load %s") % filename);
    }
}

void Texture::draw(Size draw_size, double opacity) {
    glPushMatrix();
    Displaying::GL::Color4f(1.0, 1.0, 1.0, opacity);
    glBindTexture(GL_TEXTURE_2D, id); // Bind 3
    glBegin(GL_QUADS);
        glTexCoord2d(0.0, 0.0);
            glVertex2d(1.0, 1.0);
        glTexCoord2d(0.0, used_height);
            glVertex2d(1.0, draw_size.h-1.0);
        glTexCoord2d(used_width, used_height);
            glVertex2d(draw_size.w-1.0, draw_size.h-1.0);
        glTexCoord2d(used_width, 0.0);
            glVertex2d(draw_size.w-1.0, 1.0);
    glEnd();
    glBindTexture(GL_TEXTURE_2D, 0); // unBind 3
    glPopMatrix();
}

void Texture::draw(Size draw_size)
{
    draw(draw_size, 1.0);
}

