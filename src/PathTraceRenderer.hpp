#pragma once

#include "3d/CameraControls.hpp"
#include "3d/Mesh.hpp"
#include "base/MulticoreLauncher.hpp"
#include "base/Random.hpp"
#include "io/File.hpp"
#include "io/ImageLodePngIO.hpp"

#include <chrono>
#include <memory>
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

    int                     m_pass;
    int                     m_bounces;
    bool                    m_bForceExit;
    bool                    m_bResidual;
    const MeshWithColors*   m_scene;
    const CameraControls*   m_camera;
    std::unique_ptr<Image>  m_image;
    RayTracer*              m_rt;
    AreaLight*              m_light;
    std::vector<AreaLight>* m_lights;
    Image*                  m_destImage;

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

    static void         pathTraceBlock(MulticoreLauncher::Task& t);
    static void         getTextureParameters(const RaycastResult& hit, Vec3f& diffuse, Vec3f& n, Vec3f& specular);
    static unsigned int getLightToSample(const Vec3f pos, const std::vector<AreaLight>* lights, Random& R);

    void updatePicture(Image* display);  // normalize by 1/w
    void checkFinish(void);
    void stop(void);
    void setNormalMapped(bool b) { m_normalMapped = b; }
    void setBilinearFiltering(bool b) { m_bilinear_filtering = b; }
    void setWhitted(bool b) { m_whitted = b; }
    void setDOF(bool b) { m_depth_of_field = b; }
    void setAAmode(bool b) { m_sobol_block = b; }
    void setAArays(int a) { m_aaNumRays = a; }
    void setLightRays(int a) { m_numLightRays = a; }
    void setFilterWidth(float a) { m_filt_width = a; }
    void setIntensity(float a) { m_intensity = a; }
    void setAttenuation(float a) { m_attenuation = a; }
    void setFocalDistance(float a) { m_focal_distance = a; }
    void setAperture(float a) { m_aperture = a; }
    void setTermination(float a) { m_termination_prob = a; }

    void resetRays() { m_TotalRays = 0; }

protected:
    unsigned __int64 m_TotalRays;
    float            m_raysPerSecond;

    MulticoreLauncher m_launcher;
    static bool       m_normalMapped;
    static float      m_filt_width;
    static int        m_aaNumRays;
    static bool       m_bilinear_filtering;
    static bool       m_whitted;
    static bool       m_depth_of_field;
    static bool       m_sobol_block;
    static float      m_intensity;
    static float      m_attenuation;
    static float      m_focal_distance;
    static float      m_aperture;
    static float      m_termination_prob;
    static int        m_numLightRays;
};

}  // namespace FW
