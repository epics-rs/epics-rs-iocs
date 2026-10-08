#include "pylon_shim.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstring>
#include <string>

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/ImageDecompressor.h>

namespace {

thread_local std::string g_last_error;

int fail(const char *what)
{
    g_last_error = what ? what : "unknown error";
    return -1;
}

/* Run `body`; any exception becomes -1 plus last_error. */
template <typename F> int guarded(F body)
{
    try {
        body();
        return 0;
    } catch (const Pylon::GenericException &e) {
        return fail(e.GetDescription());
    } catch (const std::exception &e) {
        return fail(e.what());
    } catch (...) {
        return fail("unknown exception");
    }
}

struct PixelMap {
    int shim;
    Pylon::EPixelType pylon;
};

const PixelMap kPixels[] = {
    {SHIM_PIXEL_MONO8, Pylon::PixelType_Mono8},
    {SHIM_PIXEL_MONO16, Pylon::PixelType_Mono16},
    {SHIM_PIXEL_RGB8, Pylon::PixelType_RGB8packed},
    {SHIM_PIXEL_RGB16, Pylon::PixelType_RGB16packed},
    {SHIM_PIXEL_BAYER_BG8, Pylon::PixelType_BayerBG8},
    {SHIM_PIXEL_BAYER_GB8, Pylon::PixelType_BayerGB8},
    {SHIM_PIXEL_BAYER_GR8, Pylon::PixelType_BayerGR8},
    {SHIM_PIXEL_BAYER_RG8, Pylon::PixelType_BayerRG8},
    {SHIM_PIXEL_BAYER_BG16, Pylon::PixelType_BayerBG16},
    {SHIM_PIXEL_BAYER_GB16, Pylon::PixelType_BayerGB16},
    {SHIM_PIXEL_BAYER_GR16, Pylon::PixelType_BayerGR16},
    {SHIM_PIXEL_BAYER_RG16, Pylon::PixelType_BayerRG16},
};

int to_shim_pixel(Pylon::EPixelType t)
{
    for (const PixelMap &p : kPixels)
        if (p.pylon == t) return p.shim;
    return SHIM_PIXEL_NONE;
}

bool to_pylon_pixel(int shim, Pylon::EPixelType *out)
{
    for (const PixelMap &p : kPixels)
        if (p.shim == shim) {
            *out = p.pylon;
            return true;
        }
    return false;
}

} // namespace

struct PylonCam : public Pylon::CImageEventHandler,
                  public Pylon::CConfigurationEventHandler,
                  public Pylon::CCameraEventHandler {
    Pylon::CBaslerUniversalInstantCamera camera;
    Pylon::CImageDecompressor decompressor;
    ShimCallbacks cb;
    std::atomic<int> convert_pixel{SHIM_PIXEL_NONE};
    std::atomic<int> convert_bit_align{0};
    std::atomic<int> convert_shift{0};

    void OnImageGrabbed(Pylon::CInstantCamera &, const Pylon::CGrabResultPtr &result) override
    {
        ShimFrame f;
        std::memset(&f, 0, sizeof(f));
        f.pixel = SHIM_PIXEL_NONE;
        std::string error, convert_error;
        Pylon::CPylonImage image;

        int rc = guarded([&] {
            if (!result->GrabSucceeded()) {
                error = result->GetErrorDescription().c_str();
                return;
            }
            Pylon::EPixelType pixel_type = result->GetPixelType();
            image.AttachGrabResultBuffer(result);

            /* The image may have been compressed by the camera and still be
             * compressed; a transport layer could have decompressed it. */
            Pylon::CompressionInfo_t info;
            if (decompressor.GetCompressionInfo(info, result) && info.hasCompressedImage) {
                if (info.compressionStatus != Pylon::CompressionStatus_Ok) {
                    error = "error in decompression";
                    return;
                }
                image.Release();
                decompressor.DecompressImage(image, result);
            }

            Pylon::EPixelType wanted;
            if (to_pylon_pixel(convert_pixel.load(), &wanted)) {
                try {
                    Pylon::CImageFormatConverter converter;
                    converter.OutputPixelFormat = wanted;
                    converter.OutputBitAlignment =
                        (Pylon::OutputBitAlignmentEnums)convert_bit_align.load();
                    converter.MonoConversionMethod = Pylon::MonoConversionMethod_Truncate;
                    converter.AdditionalLeftShift = convert_shift.load();
                    Pylon::CPylonImage converted;
                    converter.Convert(converted, image);
                    image = converted;
                    pixel_type = wanted;
                } catch (const Pylon::GenericException &e) {
                    convert_error = e.GetDescription();
                }
            }

            if (!image.IsValid()) {
                error = "image is invalid";
                return;
            }
            f.width = result->GetWidth();
            f.height = result->GetHeight();
            f.pixel = to_shim_pixel(pixel_type);
            f.raw_pixel_type = (int64_t)pixel_type;
            f.data = image.GetBuffer();
            f.size = image.GetImageSize();
            f.id = (int64_t)result->GetID();
            f.timestamp = result->GetTimeStamp();
            if (result->IsChunkDataAvailable())
                f.chunks = (void *)&result->GetChunkDataNodeMap();
        });
        if (rc != 0) error = g_last_error;

        f.ok = error.empty();
        f.error = error.empty() ? nullptr : error.c_str();
        f.convert_error = convert_error.empty() ? nullptr : convert_error.c_str();
        cb.on_frame(cb.user, &f);
    }

    void OnCameraDeviceRemoved(Pylon::CInstantCamera &) override { cb.on_removed(cb.user); }

    void OnCameraEvent(Pylon::CInstantCamera &, intptr_t id, GenApi::INode *) override
    {
        cb.on_event(cb.user, (int)id);
    }
};

namespace {

GenApi::INode *find_node(PylonCam *cam, int map, const char *name)
{
    if (!cam->camera.IsOpen()) return nullptr;
    GenApi::INodeMap &nm = map == SHIM_MAP_STREAM ? cam->camera.GetStreamGrabberNodeMap()
                                                  : cam->camera.GetNodeMap();
    return nm.GetNode(name);
}

template <typename T> T *node_as(PylonCam *cam, int map, const char *name)
{
    T *p = dynamic_cast<T *>(find_node(cam, map, name));
    if (!p) throw std::runtime_error(std::string("no such node: ") + name);
    return p;
}

int node_type_of(GenApi::INode *node)
{
    switch (node->GetPrincipalInterfaceType()) {
    case GenApi::intfIInteger: return SHIM_NODE_INTEGER;
    case GenApi::intfIBoolean: return SHIM_NODE_BOOLEAN;
    case GenApi::intfIEnumeration: return SHIM_NODE_ENUM;
    case GenApi::intfIFloat: return SHIM_NODE_FLOAT;
    case GenApi::intfIString: return SHIM_NODE_STRING;
    case GenApi::intfICommand: return SHIM_NODE_COMMAND;
    default: return SHIM_NODE_OTHER;
    }
}

} // namespace

extern "C" {

const char *pylon_shim_last_error(void) { return g_last_error.c_str(); }

void pylon_shim_initialize(void) { Pylon::PylonInitialize(); }

void pylon_shim_terminate(void) { Pylon::PylonTerminate(); }

void pylon_shim_version(ShimStringSink sink, void *user)
{
    sink(user, Pylon::GetPylonVersionString());
}

int pylon_shim_enumerate(ShimDeviceSink sink, void *user)
{
    return guarded([&] {
        Pylon::DeviceInfoList_t devices;
        Pylon::CTlFactory::GetInstance().EnumerateDevices(devices);
        for (const Pylon::CDeviceInfo &d : devices)
            sink(user, d.GetFriendlyName().c_str(), d.GetModelName().c_str(),
                 d.GetSerialNumber().c_str(), d.GetInterfaceID().c_str());
    });
}

PylonCam *pylon_cam_new(const ShimCallbacks *callbacks)
{
    PylonCam *cam = new PylonCam();
    cam->cb = *callbacks;
    return cam;
}

void pylon_cam_free(PylonCam *cam)
{
    if (!cam) return;
    guarded([&] { cam->camera.DestroyDevice(); });
    delete cam;
}

int pylon_cam_open(PylonCam *cam, const char *camera_id)
{
    return guarded([&] {
        std::string id = camera_id;
        Pylon::CTlFactory &factory = Pylon::CTlFactory::GetInstance();
        bool is_index = id.size() < 4 && !id.empty() &&
                        std::all_of(id.begin(), id.end(),
                                    [](unsigned char c) { return std::isdigit(c) != 0; });
        if (is_index) {
            size_t index = (size_t)std::stoi(id);
            Pylon::DeviceInfoList_t devices;
            factory.EnumerateDevices(devices);
            if (index >= devices.size())
                throw std::runtime_error("index " + id + " >= cameras found " +
                                         std::to_string(devices.size()));
            cam->camera.Attach(factory.CreateDevice(devices[index]), Pylon::Cleanup_Delete);
        } else {
            Pylon::CDeviceInfo info;
            info.SetSerialNumber(id.c_str());
            cam->camera.Attach(factory.CreateDevice(info), Pylon::Cleanup_Delete);
        }
        /* ReplaceAll also drops pylon's default AcquireContinuous
         * configuration, so opening does not rewrite AcquisitionMode. */
        cam->camera.RegisterImageEventHandler(cam, Pylon::RegistrationMode_ReplaceAll,
                                              Pylon::Cleanup_None);
        cam->camera.RegisterConfiguration(cam, Pylon::RegistrationMode_ReplaceAll,
                                          Pylon::Cleanup_None);
        cam->camera.GrabCameraEvents = true;
        cam->camera.Open();
    });
}

void pylon_cam_close(PylonCam *cam)
{
    guarded([&] { cam->camera.Close(); });
}

int pylon_cam_is_open(PylonCam *cam) { return cam->camera.IsOpen() ? 1 : 0; }

int pylon_cam_sfnc_major(PylonCam *cam)
{
    int major = 0;
    guarded([&] { major = (int)cam->camera.GetSfncVersion().getMajor(); });
    return major;
}

int pylon_cam_register_event(PylonCam *cam, const char *node_name, int event_id, int append)
{
    return guarded([&] {
        cam->camera.RegisterCameraEventHandler(
            cam, node_name, event_id,
            append ? Pylon::RegistrationMode_Append : Pylon::RegistrationMode_ReplaceAll,
            Pylon::Cleanup_None, Pylon::CameraEventAvailability_Optional);
    });
}

int pylon_cam_start(PylonCam *cam, int64_t count)
{
    /* Compression may be unsupported or unconfigured. */
    try {
        cam->decompressor.SetCompressionDescriptor(cam->camera.GetNodeMap());
    } catch (const Pylon::GenericException &) {
    }
    return guarded([&] {
        if (count < 0)
            cam->camera.StartGrabbing(Pylon::GrabStrategy_OneByOne,
                                      Pylon::GrabLoop_ProvidedByInstantCamera);
        else
            cam->camera.StartGrabbing((size_t)count, Pylon::GrabStrategy_OneByOne,
                                      Pylon::GrabLoop_ProvidedByInstantCamera);
    });
}

int pylon_cam_stop(PylonCam *cam)
{
    return guarded([&] { cam->camera.StopGrabbing(); });
}

void pylon_cam_set_convert(PylonCam *cam, int pixel, int bit_align, int shift_bits)
{
    cam->convert_pixel.store(pixel);
    cam->convert_bit_align.store(bit_align);
    cam->convert_shift.store(shift_bits);
}

int pylon_node_type(PylonCam *cam, int map, const char *name)
{
    int type = SHIM_NODE_ABSENT;
    guarded([&] {
        GenApi::INode *node = find_node(cam, map, name);
        if (node) type = node_type_of(node);
    });
    return type;
}

int pylon_node_access(PylonCam *cam, int map, const char *name, int *available, int *readable,
                      int *writable)
{
    *available = *readable = *writable = 0;
    return guarded([&] {
        GenApi::INode *node = find_node(cam, map, name);
        if (!node) throw std::runtime_error(std::string("no such node: ") + name);
        *available = GenApi::IsAvailable(node) ? 1 : 0;
        *readable = GenApi::IsReadable(node) ? 1 : 0;
        *writable = GenApi::IsWritable(node) ? 1 : 0;
    });
}

int pylon_node_get_int(PylonCam *cam, int map, const char *name, int64_t *value, int64_t *min,
                       int64_t *max, int64_t *inc)
{
    return guarded([&] {
        GenApi::IInteger *p = node_as<GenApi::IInteger>(cam, map, name);
        if (value) *value = p->GetValue();
        if (min) *min = p->GetMin();
        if (max) *max = p->GetMax();
        if (inc) *inc = p->GetInc();
    });
}

int pylon_node_set_int(PylonCam *cam, int map, const char *name, int64_t value)
{
    return guarded([&] { node_as<GenApi::IInteger>(cam, map, name)->SetValue(value); });
}

int pylon_node_get_float(PylonCam *cam, int map, const char *name, double *value, double *min,
                         double *max)
{
    return guarded([&] {
        GenApi::IFloat *p = node_as<GenApi::IFloat>(cam, map, name);
        if (value) *value = p->GetValue();
        if (min) *min = p->GetMin();
        if (max) *max = p->GetMax();
    });
}

int pylon_node_set_float(PylonCam *cam, int map, const char *name, double value)
{
    return guarded([&] { node_as<GenApi::IFloat>(cam, map, name)->SetValue(value); });
}

int pylon_node_get_bool(PylonCam *cam, int map, const char *name, int *value)
{
    return guarded(
        [&] { *value = node_as<GenApi::IBoolean>(cam, map, name)->GetValue() ? 1 : 0; });
}

int pylon_node_set_bool(PylonCam *cam, int map, const char *name, int value)
{
    return guarded([&] { node_as<GenApi::IBoolean>(cam, map, name)->SetValue(value != 0); });
}

int pylon_node_get_enum(PylonCam *cam, int map, const char *name, int64_t *value)
{
    return guarded(
        [&] { *value = node_as<GenApi::IEnumeration>(cam, map, name)->GetIntValue(); });
}

int pylon_node_set_enum(PylonCam *cam, int map, const char *name, int64_t value)
{
    return guarded([&] { node_as<GenApi::IEnumeration>(cam, map, name)->SetIntValue(value); });
}

int pylon_node_get_enum_symbolic(PylonCam *cam, int map, const char *name, ShimStringSink sink,
                                 void *user)
{
    return guarded([&] {
        GenICam::gcstring s = node_as<GenApi::IEnumeration>(cam, map, name)->ToString();
        sink(user, s.c_str());
    });
}

int pylon_node_enum_entries(PylonCam *cam, int map, const char *name, int settable_only,
                            ShimEnumSink sink, void *user)
{
    return guarded([&] {
        GenApi::IEnumeration *p = node_as<GenApi::IEnumeration>(cam, map, name);
        if (settable_only) {
            Pylon::CEnumParameter parameter(p);
            Pylon::StringList_t symbolics;
            parameter.GetSettableValues(symbolics);
            for (const Pylon::String_t &s : symbolics) {
                GenApi::IEnumEntry *entry = parameter.GetEntryByName(s);
                if (entry) sink(user, entry->GetSymbolic().c_str(), entry->GetValue());
            }
        } else {
            GenApi::NodeList_t entries;
            p->GetEntries(entries);
            for (GenApi::INode *n : entries) {
                GenApi::IEnumEntry *entry = dynamic_cast<GenApi::IEnumEntry *>(n);
                if (entry) sink(user, entry->GetSymbolic().c_str(), entry->GetValue());
            }
        }
    });
}

int pylon_node_get_string(PylonCam *cam, int map, const char *name, ShimStringSink sink,
                          void *user)
{
    return guarded([&] {
        GenICam::gcstring s = node_as<GenApi::IString>(cam, map, name)->GetValue();
        sink(user, s.c_str());
    });
}

int pylon_node_set_string(PylonCam *cam, int map, const char *name, const char *value)
{
    return guarded([&] { node_as<GenApi::IString>(cam, map, name)->SetValue(value); });
}

int pylon_node_execute(PylonCam *cam, int map, const char *name)
{
    return guarded([&] { node_as<GenApi::ICommand>(cam, map, name)->Execute(); });
}

void pylon_frame_chunks(void *chunks, ShimChunkSink sink, void *user)
{
    if (!chunks) return;
    const GenApi::INodeMap &node_map = *(const GenApi::INodeMap *)chunks;
    GenApi::NodeList_t nodes;
    node_map.GetNodes(nodes);
    for (GenApi::INode *node : nodes) {
        /* Chunk parameters are read-only and carry the "Chunk" prefix. */
        if (node->GetAccessMode() != GenApi::RO) continue;
        if (std::strncmp(node->GetName().c_str(), "Chunk", 5) != 0) continue;
        guarded([&] {
            GenICam::gcstring name_s = node->GetName();
            GenICam::gcstring display_s = node->GetDisplayName();
            const char *name = name_s.c_str();
            const char *display = display_s.c_str();
            switch (node->GetPrincipalInterfaceType()) {
            case GenApi::intfIInteger:
                sink(user, name, display, SHIM_NODE_INTEGER,
                     dynamic_cast<GenApi::IInteger *>(node)->GetValue(), 0.0, nullptr);
                break;
            case GenApi::intfIFloat:
                sink(user, name, display, SHIM_NODE_FLOAT, 0,
                     dynamic_cast<GenApi::IFloat *>(node)->GetValue(), nullptr);
                break;
            case GenApi::intfIEnumeration:
                sink(user, name, display, SHIM_NODE_ENUM,
                     dynamic_cast<GenApi::IEnumeration *>(node)->GetIntValue(), 0.0, nullptr);
                break;
            case GenApi::intfIBoolean:
                sink(user, name, display, SHIM_NODE_BOOLEAN,
                     dynamic_cast<GenApi::IBoolean *>(node)->GetValue() ? 1 : 0, 0.0, nullptr);
                break;
            case GenApi::intfIString: {
                GenICam::gcstring s = dynamic_cast<GenApi::IString *>(node)->GetValue();
                sink(user, name, display, SHIM_NODE_STRING, 0, 0.0, s.c_str());
                break;
            }
            default:
                break;
            }
        });
    }
}

} // extern "C"
