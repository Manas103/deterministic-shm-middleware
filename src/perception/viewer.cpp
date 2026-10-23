// The OpenGL viewer: attaches to the shared-memory mailbox perception_node
// writes, uploads each frame's RGBA bytes into a GL texture, draws it into
// an off-screen framebuffer (headless EGL, no display, no window system),
// reads the framebuffer back, and checksums it against the producer's own
// checksum of the same bytes. A match proves the viewer actually read the
// pipeline's real output, not a stand-in.
//
// This machine's EGL/GL stack is Mesa's software/D3D12 translation path
// (confirmed by GL_RENDERER at startup), not the NVIDIA driver CUDA runs
// on, so there is no real CUDA-OpenGL interop available here (see README
// Findings); the image bytes cross from the CUDA pipeline to this process
// through the same host-shared-memory mailbox every other inter-process
// handoff in this repo uses, not a zero-copy GPU-to-GPU path.
#include <EGL/egl.h>
#include <GL/gl.h>
#include <GL/glext.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

#include "shmmw/perception/frame_mailbox.hpp"
#include "shmmw/shm_segment.hpp"

using namespace shmmw::perception;

static const char* kSegmentName = "/shmmw_perception_mailbox";

// Core-profile FBO functions are not in the legacy GL/gl.h this machine's
// Mesa headers ship (that header only promises GL 1.x); they have to be
// resolved at runtime via eglGetProcAddress, the standard way to reach
// anything past GL 1.x on Linux regardless of which loader you use.
static PFNGLGENFRAMEBUFFERSPROC glGenFramebuffers_;
static PFNGLBINDFRAMEBUFFERPROC glBindFramebuffer_;
static PFNGLFRAMEBUFFERTEXTURE2DPROC glFramebufferTexture2D_;

static bool load_gl_functions() {
    glGenFramebuffers_ = (PFNGLGENFRAMEBUFFERSPROC)eglGetProcAddress("glGenFramebuffers");
    glBindFramebuffer_ = (PFNGLBINDFRAMEBUFFERPROC)eglGetProcAddress("glBindFramebuffer");
    glFramebufferTexture2D_ = (PFNGLFRAMEBUFFERTEXTURE2DPROC)eglGetProcAddress("glFramebufferTexture2D");
    return glGenFramebuffers_ && glBindFramebuffer_ && glFramebufferTexture2D_;
}

static bool init_headless_gl(EGLDisplay* out_dpy) {
    EGLDisplay dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (dpy == EGL_NO_DISPLAY) return false;
    EGLint major, minor;
    if (!eglInitialize(dpy, &major, &minor)) return false;

    EGLint cfg_attribs[] = {
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8, EGL_NONE,
    };
    EGLConfig cfg;
    EGLint num_cfg;
    if (!eglChooseConfig(dpy, cfg_attribs, &cfg, 1, &num_cfg) || num_cfg == 0) return false;
    eglBindAPI(EGL_OPENGL_API);
    EGLint pbuf_attribs[] = {EGL_WIDTH, kImgW, EGL_HEIGHT, kImgH, EGL_NONE};
    EGLSurface surf = eglCreatePbufferSurface(dpy, cfg, pbuf_attribs);
    if (surf == EGL_NO_SURFACE) return false;
    EGLContext ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, nullptr);
    if (ctx == EGL_NO_CONTEXT) return false;
    if (!eglMakeCurrent(dpy, surf, surf, ctx)) return false;
    *out_dpy = dpy;
    return true;
}

int main(int argc, char** argv) {
    int n_frames = 20;
    if (argc > 1) n_frames = atoi(argv[1]);

    EGLDisplay dpy;
    if (!init_headless_gl(&dpy)) {
        fprintf(stderr, "headless EGL/GL init failed\n");
        return 1;
    }
    if (!load_gl_functions()) {
        fprintf(stderr, "failed to resolve FBO entry points via eglGetProcAddress\n");
        return 1;
    }
    fprintf(stderr, "GL_RENDERER: %s\n", (const char*)glGetString(GL_RENDERER));

    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    GLuint fbo;
    glGenFramebuffers_(1, &fbo);

    auto seg = shmmw::ShmSegment::attach(kSegmentName, sizeof(FrameMailbox));
    auto* mailbox = reinterpret_cast<FrameMailbox*>(seg.data());

    std::vector<uint8_t> readback(kImgW * kImgH * 4);
    int matches = 0;

    for (int f = 0; f < n_frames; ++f) {
        uint64_t target = (uint64_t)f + 1;
        int waited_ms = 0;
        while (mailbox->write_seq.load(std::memory_order_acquire) < target) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if (++waited_ms > 5000) {
                fprintf(stderr, "no frame %d from producer within 5s\n", f);
                return 1;
            }
        }

        uint64_t producer_checksum = mailbox->producer_checksum;
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, kImgW, kImgH, 0, GL_RGBA, GL_UNSIGNED_BYTE, mailbox->image);

        glBindFramebuffer_(GL_FRAMEBUFFER, fbo);
        glFramebufferTexture2D_(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
        glViewport(0, 0, kImgW, kImgH);
        glReadPixels(0, 0, kImgW, kImgH, GL_RGBA, GL_UNSIGNED_BYTE, readback.data());

        uint64_t viewer_checksum = additive_checksum(readback.data(), readback.size());
        bool match = (viewer_checksum == producer_checksum);
        if (match) matches++;
        printf("frame %d: producer_checksum=%llu viewer_checksum=%llu %s\n", f,
               (unsigned long long)producer_checksum, (unsigned long long)viewer_checksum,
               match ? "MATCH" : "MISMATCH");

        mailbox->read_seq.store(target, std::memory_order_release);
    }

    printf("viewer: %d of %d frames matched the producer's checksum\n", matches, n_frames);
    return (matches == n_frames) ? 0 : 1;
}
