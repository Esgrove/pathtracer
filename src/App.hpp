#pragma once

#include "gui/Window.hpp"
#include "gui/CommonControls.hpp"
#include "gui/Image.hpp"
#include "3d/CameraControls.hpp"
#include "gpu/Buffer.hpp"

#include <vector>
#include <memory>

#include "RayTracer.hpp"

#include "AreaLight.hpp"
#include "PathTraceRenderer.hpp"

namespace FW {

//------------------------------------------------------------------------

class App : public Window::Listener, public CommonControls::StateObject
{
private:
    enum Action
    {
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

    enum CullMode
    {
        CullMode_None = 0,
        CullMode_CW,
        CullMode_CCW,
    };

    struct RayVertex
    {
        Vec3f       pos;
        U32         color;
    };

	enum bvh_build_method { None, SAH };
	enum SamplingType { AO_sampling, AA_sampling };
	// this structure holds the necessary arguments when rendering using command line parameters
	struct {
		bool batch_render;
		int spp;					// samples per pixel to use
		SamplingType sample_type;	// AO or AA sampling; AO includes one extra sample for the primary ray
		bool output_images;			// might be useful to compare images with the example
		bool use_textures;			// whether or not textures are used
		bool use_arealights;		// whether or not area light sampling is used
		bool enable_reflections;	// whether to compute reflections in whitted integrator
		bool use_sah;
		float ao_length;			
	} m_settings;
	
	struct {
		std::string state_name;										// filenames of the state and scene files
		std::string scene_name;
		int rayCount;
		int build_time, trace_time;

	} m_results;

public:
					 App			(std::vector<std::string>& cmd_args);
    virtual         ~App            (void);

    virtual bool    handleEvent     (const Window::Event& ev);
    virtual void    readState       (StateDump& d);
    virtual void    writeState      (StateDump& d) const;

private:
	void			process_args	(std::vector<std::string>& args);

    void            waitKey         (void);
    void            renderFrame     (GLContext* gl);
    void            renderScene     (GLContext* gl, const Mat4f& worldToCamera, const Mat4f& projection);
    void            loadMesh        (const String& fileName);
    void            saveMesh        (const String& fileName);
    void            loadRayDump     (const String& fileName);

    static void     downscaleTextures(MeshBase* mesh);
    static void     chopBehindPlane (MeshBase* mesh, const Vec4f& pleq);

    static bool		fileExists		(const String& fileName);

	void			constructTracer(void);

	void			blitRttToScreen(GLContext* gl);

private:
                    App             (const App&); // forbidden
    App&            operator=       (const App&); // forbidden

private:
    Window          m_window;
    CommonControls  m_commonCtrl;
    CameraControls  m_cameraCtrl;

    Action          m_action;
    String          m_meshFileName;
    CullMode        m_cullMode;

    std::unique_ptr<RayTracer>			m_rt;
	std::vector<Vec3f>				    m_rtVertexPositions; // kept only for MD5 checksums
    std::vector<RTTriangle>				m_rtTriangles;

    std::unique_ptr<MeshWithColors>     m_mesh;
	std::unique_ptr<AreaLight>          m_areaLight;
    std::unique_ptr<PathTraceRenderer>	m_pathtrace_renderer;
	std::unique_ptr<std::vector<AreaLight>>	m_areaLights;
	unsigned int						m_light_index;
	int									m_numBounces;
	float								m_lightSize;
	Timer								m_updateClock;

	bool								m_RTMode;
	bool								m_useRussianRoulette;
	bool								m_normalMapped;
	bool								m_bilinear;
	bool								m_whitted;
	bool								m_use_sah;
	bool								m_depth_of_field;
	bool								m_sobol_block;
	int									m_numAARays;
	int									m_numLightRays;
	float								m_filter_width;
	float                               m_light_intensity;
	float                               m_saturation;
	float                               m_aperture;
	float                               m_focal_distance;
	float								m_termination;
	float								m_emission_red;
	float								m_emission_green;
	float								m_emission_blue;
	
	bool								clear_on_next_frame = false;
	Mat4f								previous_camera = Mat4f(0);
	Image								m_img;
	int									m_numDebugPathCount = 1;
	int									m_currentVisualizationIndex = 0;
	float								m_visualizationAlpha = 1;
	bool								m_playbackVisualization = false;
	bool								m_clearVisualization = false;

	std::vector<PathVisualizationNode> m_visualization;

};


//------------------------------------------------------------------------
}
