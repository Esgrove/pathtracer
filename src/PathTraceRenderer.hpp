#pragma once

#include "3d/CameraControls.hpp"
#include "3d/Mesh.hpp"
#include "base/MulticoreLauncher.hpp"
#include "base/Random.hpp"
#include "io/File.hpp"
#include "io/ImageLodePngIO.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace FW {

typedef Mesh<VertexPNTC> MeshWithColors;

class RayTracer;
class Image;
class AreaLight;
struct RaycastResult;
struct RTTriangle;

// Defines a block which is rendered by a single thread as a single task.
struct PathTracerBlock {
    int m_x;       // X coordinate of the leftmost pixel of the block.
    int m_y;       // Y coordinate of the topmost pixel of the block.
    int m_width;   // Pixel width of the block.
    int m_height;  // Pixel height of the block.
};

struct PathTracerContext {
    PathTracerContext();
    ~PathTracerContext();

    std::vector<PathTracerBlock> m_blocks;  // Render blocks for rendering tasks. Index by .idx

    AreaLight*            m_light;
    const CameraControls* m_camera;
    const MeshWithColors* m_scene;
    Image*                m_destImage;
    RayTracer*            m_rt;

    bool m_bForceExit;
    bool m_bResidual;
    int  m_bounces;
    int  m_pass;

    std::unique_ptr<Image>  m_image;
    std::vector<AreaLight>* m_lights;

    std::chrono::steady_clock::time_point start;  // time keeping for stats
};

class PathVisualizationLine {
public:
    Vec3f start, stop, color;

    PathVisualizationLine(const Vec3f& start, const Vec3f& stop, const Vec3f color = Vec3f(1))
        : start(start)
        , stop(stop)
        , color(color) {}
};

class PathVisualizationLabel {
public:
    std::string text;
    Vec3f       position;

    PathVisualizationLabel(std::string text, Vec3f position) : text(text), position(position) {}
};

class PathVisualizationNode {
public:
    std::vector<PathVisualizationLabel> labels;
    std::vector<PathVisualizationLine>  lines;
};

// This class contains functionality to render pictures using a ray tracer
class PathTraceRenderer {
public:
    PathTraceRenderer();
    ~PathTraceRenderer();

    // whether or not visualization data should be generated
    static bool       debugVis;
    PathTracerContext m_context;

    // are we still processing?
    bool isRunning(void) const { return m_launcher.getNumTasks() > 0; }

    // negative #bounces = -N means start russian roulette from Nth bounce
    // positive N means always trace up to N bounces
    void startPathTracingProcess(const MeshWithColors* scene, std::vector<AreaLight>* lights, RayTracer* rt, Image* dest, int bounces, const CameraControls& camera);

    static Vec3f tracePath(float x, float y, PathTracerContext& ctx, int samplerBase, Random& rnd, std::vector<PathVisualizationNode>& visualization);
    static Vec3f traceWhitted(float x, float y, PathTracerContext& ctx, int samplerBase, Random& rnd, std::vector<PathVisualizationNode>& visualization);

    static void pathTraceBlock(MulticoreLauncher::Task& t);
    static void getTextureParameters(const RaycastResult& hit, Vec3f& diffuse, Vec3f& n, Vec3f& specular);
    static int  getLightToSample(const Vec3f pos, const std::vector<AreaLight>* lights, Random& R);

    void updatePicture(Image* display);  // normalize by 1/w
    void checkFinish(void);
    void stop(void);
    void setNormalMapped(bool b) { m_normalMapped = b; }
    void setBilinearFiltering(bool b) { m_bilinearFiltering = b; }
    void setWhitted(bool b) { m_whitted = b; }
    void setDOF(bool b) { m_depthOfField = b; }
    void setAAmode(bool b) { m_sobolBlock = b; }
    void setAArays(int a) { m_NumAaRays = a; }
    void setLightRays(int a) { m_numLightRays = a; }
    void setFilterWidth(float a) { m_filterWidth = a; }
    void setIntensity(float a) { m_intensity = a; }
    void setAttenuation(float a) { m_attenuation = a; }
    void setFocalDistance(float a) { m_focalDistance = a; }
    void setAperture(float a) { m_aperture = a; }
    void setTermination(float a) { m_terminationProbability = a; }
    void resetRays() { m_TotalRays = 0; }

protected:
    MulticoreLauncher m_launcher;

    float            m_raysPerSecond;
    static bool      m_bilinearFiltering;
    static bool      m_depthOfField;
    static bool      m_normalMapped;
    static bool      m_sobolBlock;
    static bool      m_whitted;
    static float     m_aperture;
    static float     m_attenuation;
    static float     m_filterWidth;
    static float     m_focalDistance;
    static float     m_intensity;
    static float     m_terminationProbability;
    static int       m_NumAaRays;
    static int       m_numLightRays;
    unsigned __int64 m_TotalRays;
};

}  // namespace FW
