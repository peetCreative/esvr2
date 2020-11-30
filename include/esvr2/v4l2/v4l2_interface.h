#pragma once

#include <string>
#include <vector>
#include <stdexcept>
#include <sys/types.h>
#include <linux/videodev2.h>

namespace v4l2 {

class V4L2Interface {
    public:
        struct Buffer {
            void   *start;
            size_t  length;
        };

        class IOError : public std::runtime_error {
            public:
                explicit IOError(const std::string &what_arg) : std::runtime_error(what_arg) {}
                explicit IOError(const char *what_arg) : std::runtime_error(what_arg) {}
        };

        V4L2Interface();
        V4L2Interface(const std::string &videoDevice, const std::vector<uint32_t>& resolution, const std::vector<uint32_t>& framerate);
        ~V4L2Interface();

        void open(const std::string &videoDevice, const std::vector<uint32_t>& resolution, const std::vector<uint32_t>& framerate);
        void close();

        Buffer& getImage();
        void releaseImage();

        bool isOpen() const;
        uint32_t getWidth() const;
        uint32_t getHeight() const;
        uint32_t getColorFormat() const;
        uint32_t getBytesPerPixel() const;

    private:
        std::vector<Buffer> buffers;

        int device_fd = 0;
        struct v4l2_buffer current_buffer;
        uint32_t width,
                 height,
                 color_format,
                 bytes_per_pixel;
};

}
