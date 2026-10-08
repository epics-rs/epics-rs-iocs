/* C ABI over the Basler pylon C++ SDK. No exception crosses this boundary:
 * a failing call returns -1 and leaves its text in pylon_shim_last_error(). */
#ifndef PYLON_SHIM_H
#define PYLON_SHIM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PylonCam PylonCam;

enum { SHIM_MAP_CAMERA = 0, SHIM_MAP_STREAM = 1 };

enum {
    SHIM_NODE_ABSENT = -1,
    SHIM_NODE_INTEGER = 0,
    SHIM_NODE_BOOLEAN = 1,
    SHIM_NODE_ENUM = 2,
    SHIM_NODE_FLOAT = 3,
    SHIM_NODE_STRING = 4,
    SHIM_NODE_COMMAND = 5,
    SHIM_NODE_OTHER = 6
};

enum {
    SHIM_PIXEL_NONE = -1,
    SHIM_PIXEL_MONO8 = 0,
    SHIM_PIXEL_MONO16,
    SHIM_PIXEL_RGB8,
    SHIM_PIXEL_RGB16,
    SHIM_PIXEL_BAYER_BG8,
    SHIM_PIXEL_BAYER_GB8,
    SHIM_PIXEL_BAYER_GR8,
    SHIM_PIXEL_BAYER_RG8,
    SHIM_PIXEL_BAYER_BG16,
    SHIM_PIXEL_BAYER_GB16,
    SHIM_PIXEL_BAYER_GR16,
    SHIM_PIXEL_BAYER_RG16
};

typedef struct {
    int ok;
    const char *error;         /* set when !ok */
    const char *convert_error; /* set when the requested conversion failed */
    uint32_t width;
    uint32_t height;
    int pixel;               /* SHIM_PIXEL_*, NONE when not representable */
    int64_t raw_pixel_type;  /* Pylon::EPixelType, for diagnostics */
    const void *data;
    size_t size;
    int64_t id;
    uint64_t timestamp;
    void *chunks; /* for pylon_frame_chunks; NULL when there is no chunk data */
} ShimFrame;

typedef struct {
    void *user;
    void (*on_frame)(void *user, const ShimFrame *frame);
    void (*on_event)(void *user, int event_id);
    void (*on_removed)(void *user);
} ShimCallbacks;

typedef void (*ShimStringSink)(void *user, const char *s);
typedef void (*ShimEnumSink)(void *user, const char *symbolic, int64_t value);
typedef void (*ShimDeviceSink)(void *user, const char *friendly_name, const char *model,
                               const char *serial, const char *interface_id);
typedef void (*ShimChunkSink)(void *user, const char *name, const char *display_name, int type,
                              int64_t ival, double dval, const char *sval);

const char *pylon_shim_last_error(void);
void pylon_shim_initialize(void);
void pylon_shim_terminate(void);
void pylon_shim_version(ShimStringSink sink, void *user);
int pylon_shim_enumerate(ShimDeviceSink sink, void *user);

PylonCam *pylon_cam_new(const ShimCallbacks *callbacks);
void pylon_cam_free(PylonCam *cam);
/* camera_id: fewer than 4 digits is an index into the enumeration, anything
 * else a serial number. */
int pylon_cam_open(PylonCam *cam, const char *camera_id);
void pylon_cam_close(PylonCam *cam);
int pylon_cam_is_open(PylonCam *cam);
int pylon_cam_sfnc_major(PylonCam *cam);
int pylon_cam_register_event(PylonCam *cam, const char *node_name, int event_id, int append);
/* count < 0 grabs until stopped. */
int pylon_cam_start(PylonCam *cam, int64_t count);
int pylon_cam_stop(PylonCam *cam);
/* pixel: SHIM_PIXEL_NONE leaves frames as the camera sent them. */
void pylon_cam_set_convert(PylonCam *cam, int pixel, int bit_align, int shift_bits);

int pylon_node_type(PylonCam *cam, int map, const char *name);
int pylon_node_access(PylonCam *cam, int map, const char *name, int *available, int *readable,
                      int *writable);
int pylon_node_get_int(PylonCam *cam, int map, const char *name, int64_t *value, int64_t *min,
                       int64_t *max, int64_t *inc);
int pylon_node_set_int(PylonCam *cam, int map, const char *name, int64_t value);
int pylon_node_get_float(PylonCam *cam, int map, const char *name, double *value, double *min,
                         double *max);
int pylon_node_set_float(PylonCam *cam, int map, const char *name, double value);
int pylon_node_get_bool(PylonCam *cam, int map, const char *name, int *value);
int pylon_node_set_bool(PylonCam *cam, int map, const char *name, int value);
int pylon_node_get_enum(PylonCam *cam, int map, const char *name, int64_t *value);
int pylon_node_set_enum(PylonCam *cam, int map, const char *name, int64_t value);
int pylon_node_get_enum_symbolic(PylonCam *cam, int map, const char *name, ShimStringSink sink,
                                 void *user);
/* settable_only: only the entries that can be set right now. */
int pylon_node_enum_entries(PylonCam *cam, int map, const char *name, int settable_only,
                            ShimEnumSink sink, void *user);
int pylon_node_get_string(PylonCam *cam, int map, const char *name, ShimStringSink sink,
                          void *user);
int pylon_node_set_string(PylonCam *cam, int map, const char *name, const char *value);
int pylon_node_execute(PylonCam *cam, int map, const char *name);

/* Valid only inside on_frame, with the frame's `chunks`. */
void pylon_frame_chunks(void *chunks, ShimChunkSink sink, void *user);

#ifdef __cplusplus
}
#endif

#endif
