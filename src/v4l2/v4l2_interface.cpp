#include "v4l2/v4l2_interface.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <csignal>
#include <cstring>
#include <cerrno>

#include <map>
#include <algorithm>

#define MAX_DEVICES 64
#define MAX_FORMATS 64
#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace v4l2 {

inline const std::string colorFormatToName(const uint32_t& code) {
    char buf[8];
    buf[0] = (code >>  0) & 0x7f;
    buf[1] = (code >>  8) & 0x7f;
    buf[2] = (code >> 16) & 0x7f;
    buf[3] = (code >> 24) & 0x7f;
    buf[4] = '\0';
    if(code & (1 << 31)) {
        buf[4] = '-';
        buf[5] = 'B';
        buf[6] = 'E';
        buf[7] = '\0';
    }
    return std::string(buf);
}

// List of prefered format identifiers and the corresponding pixel size in
// bytes. The storz endoscope can output both RGB and YUV colorspace, but RGB
// does not need to be converted for displaying and is therefore higher in the
// list.
const std::vector<std::pair<uint32_t, uint32_t>> prefered_formats({
        //// these can be displayed without conversation
        //// TODO: currently there are display errors with these
        //{V4L2_PIX_FMT_XRGB32,    4}, // padded rgb
        //{V4L2_PIX_FMT_ARGB32,    4}, // rgb with alpha
        //{V4L2_PIX_FMT_RGB32,     4}, // deprecated rgb with alpha

        // these have to be padded and/or pixel-swapped for display
        {V4L2_PIX_FMT_RGB24,     3}, // other common rgb formats
        {V4L2_PIX_FMT_RGB565,    2},
        {V4L2_PIX_FMT_RGB555,    2},

        {V4L2_PIX_FMT_BGR24,     3}, // reversed rgb
        {V4L2_PIX_FMT_XBGR32,    4},
        {V4L2_PIX_FMT_ABGR32,    4},
        {V4L2_PIX_FMT_BGR32,     4},

        {V4L2_PIX_FMT_GREY,      1}, // gray scale images
        {V4L2_PIX_FMT_Y16,       2},
        {V4L2_PIX_FMT_Y16_BE,    2},

        // luminance and chrominance formats that have to be converted into
        // the rgb colorspace for displaying
        {V4L2_PIX_FMT_YUYV,      2},
        {V4L2_PIX_FMT_UYVY,      2},

        {V4L2_PIX_FMT_YVU420,    2}, // actually 1.5 bytes per pixel, but
        {V4L2_PIX_FMT_YUV420,    2}, // that doesn't fit into an integer
        {V4L2_PIX_FMT_NV12,      2},
        {V4L2_PIX_FMT_NV21,      2},
});


V4L2Interface::V4L2Interface() {
}

V4L2Interface::V4L2Interface(const std::string& videoDevice, const std::vector<uint32_t>& resolution, const std::vector<uint32_t>& framerate) {
    open(videoDevice, resolution, framerate);
}

V4L2Interface::~V4L2Interface() {
    if(device_fd != 0)
        close();
}

void V4L2Interface::open(const std::string& videoDevice, const std::vector<uint32_t>& resolution, const std::vector<uint32_t>& framerate) {
    (void) resolution;
    // open V4L device
    if ((device_fd = ::open(videoDevice.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0)) == -1) {
        device_fd = 0;
        throw IOError("Error: failed to open V4L interface " + videoDevice + ": " +
                std::string(strerror(errno)));
    }

    // if any error occures, the device descriptor will be closed in the catch
    try {
        // query capabilities
        struct v4l2_capability cap;
        CLEAR(cap);

        if (-1 == TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_QUERYCAP, &cap)))
            throw IOError("Error: Failed to query capabilities on V4L device");

        // check if the device is a valid capture device
        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
            throw IOError("Error: Invalid V4L device");

        // check if the device supports streaming
        if (!(cap.capabilities & V4L2_CAP_STREAMING))
            throw IOError("Error: Device does not support streaming");

        // remove any previous cropping
        struct v4l2_cropcap cropcap;
        struct v4l2_crop crop;
        CLEAR(cropcap);

        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (0 == TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_CROPCAP, &cropcap))) {
            crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            crop.c = cropcap.defrect; /* reset to default */

            if (-1 == TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_S_CROP, &crop)))
                printf("Warning: Failed to reset cropping area, ignoring!\n");
        }


        // query supportet color formats
        color_format = 0;
        std::vector<uint32_t> supported_formats;
        struct v4l2_fmtdesc fmtdesc;
        CLEAR(fmtdesc);
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        while(0 == TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_ENUM_FMT, &fmtdesc))) {
            supported_formats.push_back(fmtdesc.pixelformat);
            fmtdesc.index++;
        }
        if(errno != EINVAL) // EINVAL is set if the end of the format list is reached
            throw IOError("Error: failed to query supportet color formats");

        for(const std::pair<uint32_t, uint32_t> &prefered_format : prefered_formats) {
            if(std::find(supported_formats.begin(), supported_formats.end(), prefered_format.first) != supported_formats.end()) {
                color_format = prefered_format.first;
                bytes_per_pixel = prefered_format.second;
                break;
            }
        }

        if(color_format == 0) {
            std::string supported;
            for(const uint32_t &format : supported_formats) {
                if(!supported.empty())
                    supported += ", ";
                supported += colorFormatToName(format);
            }
            throw IOError("The device does not support any of the prefered formats, supported formats are: " + supported);
        }

        // set image parameters
        struct v4l2_format fmt;
        CLEAR(fmt);
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (-1 == TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_G_FMT, &fmt)))
            throw IOError("Error: Failed to get format parameters");

        // resolution
        if(resolution[0] != 0 && resolution[1] != 0) {
            fmt.fmt.pix.width = resolution[0];
            fmt.fmt.pix.height = resolution[1];
        }

        // color format
        if (fmt.fmt.pix.pixelformat != color_format) {
            fmt.fmt.pix.pixelformat = color_format;
            fmt.fmt.pix.bytesperline = bytes_per_pixel * fmt.fmt.pix.width;
        }

        if (TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_TRY_FMT, &fmt))) {
            throw IOError("Error: The requested image format is not supported: " + 
                    std::to_string(fmt.fmt.pix.width) + "x" + std::to_string(fmt.fmt.pix.height) +
                    " (" + colorFormatToName(fmt.fmt.pix.pixelformat) + ")");
        } else {
            //format is supported, set format
            if(TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_S_FMT, &fmt)))
                throw IOError("Error: Failed to set pixel format");
        }

        // query the parameters again, since the driver meight have corrected them
        if (-1 == TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_G_FMT, &fmt)))
            throw IOError("Error: Failed to get format parameters");

        width = fmt.fmt.pix.width;
        height = fmt.fmt.pix.height;
        color_format = fmt.fmt.pix.pixelformat;
        bytes_per_pixel = fmt.fmt.pix.bytesperline / fmt.fmt.pix.width;

        // if requested: set the framerate
        if(framerate[0] != 0 && framerate[1] != 0) {
            struct v4l2_streamparm streamparm;
            CLEAR(streamparm);
            streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (-1 == TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_G_PARM, &streamparm)))
                throw IOError("Error: Failed to get stream parameter");

            // the framerate parameter is a frequency but the v4l2 interface
            // wants an interval, therefore numerator and denominator are exchanged
            streamparm.parm.capture.capturemode |= V4L2_CAP_TIMEPERFRAME;
            streamparm.parm.capture.timeperframe.numerator = framerate[1];
            streamparm.parm.capture.timeperframe.denominator = framerate[0];

            if (-1 == TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_S_PARM, &streamparm))) {
                // setting this framerate failed.
                // Throw an error, that lists all available rates
                struct v4l2_frmivalenum frmivalenum;
                CLEAR(frmivalenum);
                frmivalenum.pixel_format = color_format;
                frmivalenum.width = width;
                frmivalenum.height = height;
                std::string rates;
                while(0 == TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmivalenum))) {
                    if(frmivalenum.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
                        if(!rates.empty())
                            rates += ", ";
                        rates += std::to_string(frmivalenum.discrete.denominator);
                        if(frmivalenum.discrete.numerator != 1)
                            rates += "/" + std::to_string(frmivalenum.discrete.numerator);
                    } else {
                        rates = "from " + std::to_string(frmivalenum.stepwise.min.denominator) +
                            "/" + std::to_string(frmivalenum.stepwise.min.numerator) +
                            " to " + std::to_string(frmivalenum.stepwise.max.denominator) +
                            "/" + std::to_string(frmivalenum.stepwise.max.numerator) +
                            " in steps of " + std::to_string(frmivalenum.stepwise.step.denominator) +
                            "/" + std::to_string(frmivalenum.stepwise.step.numerator);
                        break;
                    }
                    frmivalenum.index++;
                }

                throw IOError("Error: Failed to set framerate. Valid rates are " + rates);
            }
        }


        // allocating buffers:
        // The most efficient way to get images from the v4l2 subsystem is via a
        // buffer ring. We requeste 4 buffers and mmap them into the local address
        // space.

        struct v4l2_requestbuffers req;
        CLEAR(req);

        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        // request buffers
        if (-1 == TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_REQBUFS, &req)))
            throw IOError("Error: Device does not support mmap");

        if (req.count < 2)
            throw IOError("Error: Insufficient buffer memory");

        buffers.resize(req.count);

        int index = 0;
        for (Buffer &buffer: buffers) {
            struct v4l2_buffer buf;

            CLEAR(buf);

            buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory      = V4L2_MEMORY_MMAP;
            buf.index       = index++;

            // query one buffer
            if (-1 == TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_QUERYBUF, &buf)))
                throw IOError("Error: Failed to initially query buffer from kernel");


            // mmap it to the local adress space
            buffer.length = buf.length;
            buffer.start = mmap(NULL, // local adress can start anywhere
                                buf.length,
                                PROT_READ | PROT_WRITE,
                                MAP_SHARED, // makes buffer available in other threads
                                device_fd,
                                buf.m.offset);

            if (MAP_FAILED == buffer.start)
                throw IOError("Error: Failed to mmap buffer");
        }

        // enqueue buffers in the driver, so images will be written into them
        for (unsigned int i = 0; i < buffers.size(); ++i) {
            struct v4l2_buffer buf;

            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (-1 == TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_QBUF, &buf)))
                throw IOError("Error: Failed to initially enqueue buffer");
        }

        // start streaming
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_STREAMON, &type)))
            throw IOError("Error: Failed to enable streaming");

    } catch(const IOError& e) {
        // make sure the device descriptor is closed an pass on the exception
        ::close(device_fd);
        device_fd = 0;
        throw;
    }
}

void V4L2Interface::close() {
    // end streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_STREAMOFF, &type))) {
        ::close(device_fd);
        device_fd = 0;
        throw IOError("Error: Failed to disable streaming");
    }

    // unmap all buffers
    for (Buffer &buffer: buffers) {
        if (-1 == munmap(buffer.start, buffer.length)) {
            ::close(device_fd);
            device_fd = 0;
            throw IOError("Error: Failed to munmap buffer");
        }
    }

    // close capture device
    ::close(device_fd);

    device_fd = 0;
}

V4L2Interface::Buffer& V4L2Interface::getImage() {
    fd_set fds;

    FD_ZERO(&fds);
    FD_SET(device_fd, &fds);

    // wait for an image to be available
    // If the select call is interrupted before the timeout is reached, the
    // call is repeated.
    int r;
    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    do {
        r = select(device_fd + 1, &fds, NULL, NULL, &tv);
    } while(r == -1 && errno == EINTR && tv.tv_sec > 0 && tv.tv_usec > 0);

    if (-1 == r)
        throw IOError("Error: Failed to wait for device: " + std::string(strerror(errno)));

    if (0 == r)
        printf("Warning: select()-call timed out on the video capture device\n");

    // dequeue the buffer descriptor for the current image
    struct v4l2_buffer new_buffer;

    CLEAR(new_buffer);

    new_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    new_buffer.memory = V4L2_MEMORY_MMAP;

    if (-1 != TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_DQBUF, &new_buffer))) {
        current_buffer = new_buffer;
    } else {
        if(errno == EAGAIN)
            printf("Warning: No buffer was in the outgoing queue, using previous buffer\n");
        else
            throw IOError("Error: Failed to get frame from device");
    }

    // return the mmap-ed buffer corresponding to the descriptor
    return buffers[current_buffer.index];
}

void V4L2Interface::releaseImage() {
    // requeue buffer that is not needed anymore
    if (-1 == TEMP_FAILURE_RETRY(ioctl(device_fd, VIDIOC_QBUF, &current_buffer)))
        throw IOError("Error: Could not requeue frame on camera");
}

bool V4L2Interface::isOpen() const{
    return (device_fd != 0);
}

uint32_t V4L2Interface::getWidth() const{
    return width;
}

uint32_t V4L2Interface::getHeight() const{
    return height;
}

uint32_t V4L2Interface::getColorFormat() const {
    return color_format;
}

uint32_t V4L2Interface::getBytesPerPixel() const {
    return bytes_per_pixel;
}

}
