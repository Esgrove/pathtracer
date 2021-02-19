#pragma once

#include "3d/CameraControls.hpp"
#include "AreaLight.hpp"
#include "PathTraceRenderer.hpp"
#include "RayTracer.hpp"
#include "gpu/Buffer.hpp"
#include "gui/CommonControls.hpp"
#include "gui/Image.hpp"
#include "gui/Window.hpp"

#include <memory>
#include <vector>

namespace FW {

class App : public Window::Listener, public CommonControls::StateObject {
private:
    enum Action {
        Action_None,

        Action_LoadMesh,
        Action_ReloadMesh,
        Action_SaveMesh,

        Action_ResetCamera,
        Action_EncodeCameraSignature,
        Action_DecodeCameraSignature,

        Action_NormalizeScale,
        Action_FlipXY,
        Action_FlipYZ,
        Action_FlipZ,

        Action_NormalizeNormals,
        Action_FlipNormals,
        Action_RecomputeNormals,

        Action_FlipTriangles,

        Action_CleanMesh,
        Action_CollapseVertices,
        Action_DupVertsPerSubmesh,
        Action_FixMaterialColors,
        Action_DownscaleTextures,
        Action_ChopBehindNear,

        Action_PathTraceMode,
        Action_PlaceLightSourceAtCamera,

        Action_AddLight,
        Action_ChangeLight
    };

    enum CullMode {
        CullMode_None = 0,
        CullMode_CW,
        CullMode_CCW,
    };

    struct RayVertex {
        Vec3f pos;
        U32   color;
    };

    enum bvh_build_method { None, SAH };
    enum SamplingType { AO_sampling, AA_sampling };
    // this structure holds the necessary arguments when rendering using command line parameters
    struct {
        bool         batch_render;
        bool         enable_reflections;  // whether to compute reflections in whitted integrator
        bool         output_images;       // might be useful to compare images with the example
        bool         use_arealights;      // whether or not area light sampling is used
        bool         use_sah;
        bool         use_textures;  // whether or not textures are used
        float        ao_length;
        int          spp;          // samples per pixel to use
        SamplingType sample_type;  // AO or AA sampling; AO includes one extra sample for the primary ray
    } m_settings;

    struct {
        std::string state_name;
        std::string scene_name;
        int         rayCount;
        int         build_time, trace_time;

    } m_results;

public:
    App(std::vector<std::string>& cmd_args);
    virtual ~App(void);

    virtual bool handleEvent(const Window::Event& ev);
    virtual void readState(StateDump& d);
    virtual void writeState(StateDump& d) const;

private:
    void process_args(std::vector<std::string>& args);

    void waitKey(void);
    void renderFrame(GLContext* gl);
    void renderScene(GLContext* gl, const Mat4f& worldToCamera, const Mat4f& projection);
    void loadMesh(const String& fileName);
    void saveMesh(const String& fileName);
    void loadRayDump(const String& fileName);

    static void downscaleTextures(MeshBase* mesh);
    static void chopBehindPlane(MeshBase* mesh, const Vec4f& pleq);

    static bool fileExists(const String& fileName);

    void constructTracer(void);

    void blitRttToScreen(GLContext* gl);

private:
    App(const App&);             // forbidden
    App& operator=(const App&);  // forbidden

private:
    Mat4f previousCamera = Mat4f(0);

    Action         m_action;
    CameraControls m_cameraCtrl;
    CommonControls m_commonCtrl;
    CullMode       m_cullMode;
    Image          m_img;
    String         m_meshFileName;
    Timer          m_updateClock;
    Window         m_window;

    std::unique_ptr<AreaLight>              m_areaLight;
    std::unique_ptr<MeshWithColors>         m_mesh;
    std::unique_ptr<PathTraceRenderer>      m_pathTraceRenderer;
    std::unique_ptr<RayTracer>              m_rt;
    std::unique_ptr<std::vector<AreaLight>> m_areaLights;
    std::vector<RTTriangle>                 m_rtTriangles;
    std::vector<Vec3f>                      m_rtVertexPositions;  // kept only for MD5 checksums

    bool  clearOnNextFrame = false;
    bool  m_bilinear;
    bool  m_clearVisualization = false;
    bool  m_depthOfField;
    bool  m_normalMapped;
    bool  m_playbackVisualization = false;
    bool  m_RTMode;
    bool  m_sobolBlock;
    bool  m_sah;
    bool  m_useRussianRoulette;
    bool  m_whitted;
    float m_lightSize;
    float m_aperture;
    float m_emissionBlue;
    float m_emissionGreen;
    float m_emissionRed;
    float m_filterWidth;
    float m_focalDistance;
    float m_lightintensity;
    float m_saturation;
    float m_termination;
    float m_visualizationAlpha = 1;
    int   m_numBounces;
    int   m_currentVisualizationIndex = 0;
    int   m_numAARays;
    int   m_numDebugPathCount = 1;
    int   m_numLightRays;
    int   m_lightIndex;

    std::vector<PathVisualizationNode> m_visualization;
};
}  // namespace FW
