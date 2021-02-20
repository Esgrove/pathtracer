#include "PathTraceRenderer.hpp"

#include "AreaLight.hpp"
#include "RayTracer.hpp"
#include "sobol.h"
#include "util.hpp"

#include <algorithm>
#include <atomic>
#include <string>
#include <valarray>

namespace FW {

bool PathTraceRenderer::debugVis = false;
bool PathTraceRenderer::m_bilinearFiltering = true;
bool PathTraceRenderer::m_depthOfField = false;
bool PathTraceRenderer::m_normalMapped = false;
bool PathTraceRenderer::m_sobolBlock = false;
bool PathTraceRenderer::m_whitted = false;
float PathTraceRenderer::m_aperture = 0.0f;
float PathTraceRenderer::m_attenuation = 1.0f;
float PathTraceRenderer::m_filterWidth = 1.0f;
float PathTraceRenderer::m_focalDistance = 0.5f;
float PathTraceRenderer::m_intensity = 1.0f;
float PathTraceRenderer::m_terminationProbability = 0.2f;
int PathTraceRenderer::m_NumAaRays = 4;
int PathTraceRenderer::m_numLightRays = 32;

const float invPi = 0.31830988618379f;

const float Pr = 0.299f;
const float Pg = 0.587f;
const float Pb = 0.114f;

Vec4f saturate(Vec4f color, float amount) {
    //  0.0 creates a black-and-white image
    //  0.5 reduces the color saturation by half
    //  1.0 causes no change
    //  2.0 doubles the color saturation

    float P = sqrt(color.x * color.x * Pr + color.y * color.y * Pg + color.z * color.z * Pb);

    color.x = P + (color.x - P) * amount;
    color.y = P + (color.y - P) * amount;
    color.z = P + (color.z - P) * amount;

    return clamp(color, 0.0f, 1.0f);
}

// 2D Gauss filter
float Gauss(float x, float y) {
    float sigma = 1.0f;
    return 1.0f / (sqrt(2.0f * FW_PI) * sigma) * exp(-1.0f * (x * x + y * y) / (2.0f * sigma * sigma));
}

Vec3f drawCosineWeightedDirection(int base, int bounce, Random R) {
    float x, y, z, a, b;

    // dimensions 2 & 3
    int dim = 2 + 4 * bounce;
    int rand = R.getS32(1, 1234);

    // Sobol
    a = 2.0f * sobol::sample(base + rand, dim) - 1.0f;
    b = 2.0f * sobol::sample(base + rand, dim + 1) - 1.0f;

    mapToDisk(a, b, x, y);

    z = sqrt(1 - x * x - y * y);

    return Vec3f(x, y, z);
}

void PathTraceRenderer::getTextureParameters(const RaycastResult& hit, Vec3f& diffuse, Vec3f& n, Vec3f& specular) {
    MeshBase::Material* mat = hit.tri->m_material;

    // interpolated texture coordinates for hit point
    Vec2f uv
        = (1 - hit.u - hit.v) * hit.tri->m_vertices[0].t + hit.u * hit.tri->m_vertices[1].t + hit.v * hit.tri->m_vertices[2].t;

    diffuse = mat->diffuse.getXYZ();
    if (mat->textures[MeshBase::TextureType_Diffuse].exists()) {
        const Texture& tex = mat->textures[MeshBase::TextureType_Diffuse];
        const Image& teximg = *tex.getImage();

        if (m_bilinearFiltering) {
            // https://en.wikipedia.org/wiki/Bilinear_filtering

            float fx = uv.x - floor(uv.x);  // {x}
            float fy = uv.y - floor(uv.y);  // {y}
            float fx1 = 1.0f - fx;
            float fy1 = 1.0f - fy;

            // weights
            float w1 = fx1 * fy1;
            float w2 = fx * fy1;
            float w3 = fx1 * fy;
            float w4 = fx * fy;

            // texel coordinates
            int x1 = (int)min(max(fx * teximg.getSize().x - 0.5f, 0.0f), teximg.getSize().x - 2.0f);  // shift 0.5 pixels
            int y1 = (int)min(max(fy * teximg.getSize().y - 0.5f, 0.0f), teximg.getSize().y - 2.0f);
            int x2 = x1 + 1;
            int y2 = y1 + 1;

            // pixel values
            Vec3f d1 = teximg.getVec4f(Vec2i(x1, y1)).getXYZ();
            Vec3f d2 = teximg.getVec4f(Vec2i(x2, y1)).getXYZ();
            Vec3f d3 = teximg.getVec4f(Vec2i(x1, y2)).getXYZ();
            Vec3f d4 = teximg.getVec4f(Vec2i(x2, y2)).getXYZ();

            // weighted sum for each color
            diffuse = Vec3f(
                d1.x * w1 + d2.x * w2 + d3.x * w3 + d4.x * w4,
                d1.y * w1 + d2.y * w2 + d3.y * w3 + d4.y * w4,
                d1.z * w1 + d2.z * w2 + d3.z * w3 + d4.z * w4);
        } else {
            Vec2i texelCoords = getTexelCoords(uv, teximg.getSize());
            diffuse = teximg.getVec4f(texelCoords).getXYZ();
        }
        // gamma correction
        diffuse = Vec3f(pow(diffuse.x, 2.2f), pow(diffuse.y, 2.2f), pow(diffuse.z, 2.2f));
    }
    // vertex normals
    Vec3f n0 = hit.tri->m_vertices[0].n;
    Vec3f n1 = hit.tri->m_vertices[1].n;
    Vec3f n2 = hit.tri->m_vertices[2].n;

    // interpolated normal
    n = normalize((1 - hit.u - hit.v) * n0 + hit.u * n1 + hit.v * n2);

    // normal texture
    if (mat->textures[MeshBase::TextureType_Normal].exists() && m_normalMapped) {
        // tangent space normal from texture
        Texture& normalTex = mat->textures[MeshBase::TextureType_Normal];
        const Image& img = *normalTex.getImage();
        Vec2i texelCoords = getTexelCoords(uv, img.getSize());
        Vec3f texture_normal = 2.0f * img.getVec4f(texelCoords).getXYZ() - 1.0f;

        // Vertices
        Vec3f v0 = hit.tri->m_vertices[0].p;
        Vec3f v1 = hit.tri->m_vertices[1].p;
        Vec3f v2 = hit.tri->m_vertices[2].p;

        // texture coordinates
        Vec2f uv0 = hit.tri->m_vertices[0].t;
        Vec2f uv1 = hit.tri->m_vertices[1].t;
        Vec2f uv2 = hit.tri->m_vertices[2].t;

        // position delta
        Vec3f deltaPos1 = v1 - v0;
        Vec3f deltaPos2 = v2 - v0;

        // UV delta
        Vec2f deltaUV1 = uv1 - uv0;
        Vec2f deltaUV2 = uv2 - uv0;

        float r = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x);

        Vec3f tangent = normalize((deltaPos1 * deltaUV2.y - deltaPos2 * deltaUV1.y) * r);
        Vec3f bitangent = normalize((deltaPos2 * deltaUV1.x - deltaPos1 * deltaUV2.x) * r);

        // Matrix
        Mat3f TBN;
        TBN.setCol(0, tangent);
        TBN.setCol(1, bitangent);
        TBN.setCol(2, n);

        // texture normal transformed to world space
        n = normalize(TBN * texture_normal);
    }
    // specular
    specular = mat->specular;
    if (mat->textures[MeshBase::TextureType_Specular].exists()) {
        Texture& specularTex = mat->textures[MeshBase::TextureType_Specular];
        const Image& img = *specularTex.getImage();
        Vec2i texelCoords = getTexelCoords(uv, img.getSize());
        specular = img.getVec4f(texelCoords).getXYZ();
    }
}

int PathTraceRenderer::getLightToSample(const Vec3f pos, const std::vector<AreaLight>* lights, Random& R) {
    auto size = (*lights).size();
    if (size <= 1) {
        // only one light
        return 0;
    }

    std::valarray<float> values(size);
    for (auto i = 0; i < size; i++) {
        // emissive power divided by distance;
        values[i] = (*lights)[i].getPower() / (pos - (*lights)[i].getPosition()).length();
    }

    // normalize
    values /= values.sum();

    float r = R.getF32(0.0f, 1.0f);
    float value = 0.0f;
    for (auto i = 0; i < size; i++) {
        value += values[i];
        if (r <= value) {
            return i;
        }
    }

    // just to be sure that something is returned always...
    return static_cast<int>(size - 1);
}

PathTracerContext::PathTracerContext()
    : m_bForceExit(false)
    , m_bResidual(false)
    , m_scene(nullptr)
    , m_rt(nullptr)
    , m_light(nullptr)
    , m_lights(nullptr)
    , m_pass(0)
    , m_bounces(0)
    , m_destImage(0)
    , m_camera(nullptr) {}

PathTracerContext::~PathTracerContext() {}

PathTraceRenderer::PathTraceRenderer() {
    m_raysPerSecond = 0.0f;
    m_TotalRays = 0;
}

PathTraceRenderer::~PathTraceRenderer() {
    stop();
}

// This function traces a single path and returns the resulting color value that will get rendered on the image
Vec3f PathTraceRenderer::tracePath(float x, float y, PathTracerContext& ctx, int samplerBase, Random& R, std::vector<PathVisualizationNode>& visualization) {
    // const MeshWithColors* scene = ctx.m_scene;
    RayTracer* rt = ctx.m_rt;
    Image* image = ctx.m_image.get();
    const CameraControls& cameraCtrl = *ctx.m_camera;
    std::vector<AreaLight>* lights = ctx.m_lights;

    // get camera orientation and projection
    Mat4f worldToCamera = cameraCtrl.getWorldToCamera();
    Mat4f projection = Mat4f::fitToView(Vec2f(-1, -1), Vec2f(2, 2), image->getSize()) * cameraCtrl.getCameraToClip();

    // inverse projection from clip space to world space
    Mat4f invP = (projection * worldToCamera).inverted();

    // point on front plane in homogeneous coordinates
    Vec4f P0(x, y, 0.0f, 1.0f);
    // point on back plane in homogeneous coordinates
    Vec4f P1(x, y, 1.0f, 1.0f);

    // apply inverse projection, divide by w to get object-space points
    Vec4f Roh = (invP * P0);
    Vec3f Ro = (Roh * (1.0f / Roh.w)).getXYZ();
    Vec4f Rdh = (invP * P1);
    Vec3f Rd = (Rdh * (1.0f / Rdh.w)).getXYZ();

    // Subtract front plane point from back plane point,
    // yields ray direction.
    // NOTE that it's not normalized: the direction Rd is defined
    // so that the segment to be traced is [Ro, Ro+Rd], i.e.,
    // intersections that come after the point Ro+Rd are to be discarded.
    Rd = Rd - Ro;

    if (m_depthOfField) {
        // focal point
        Vec3f Rn = Rd.normalized();          // normalized ray direction
        Vec3f Pn = cameraCtrl.getForward();  // focal plane normal

        // distance from ray origin to focal plane:
        // distance is increased when ray does not point directly to the focal plane normal direction
        float Pd = m_focalDistance / max(0.01f, dot(Rn, Pn));
        Vec3f Rf = Ro + Pd * Rn;  // origin + distance * direction

        // random sample on disk (aperture)
        float xr, yr, a, b;
        a = R.getF32(-1.0, 1.0f);
        b = R.getF32(-1.0, 1.0f);

        mapToDisk(a, b, xr, yr);

        // camera orientation
        Vec3f up = cameraCtrl.getOrientation().getCol(0);
        Vec3f right = cameraCtrl.getOrientation().getCol(1);

        // shift ray origin inside aperture disk and calculate new direction
        Ro += m_aperture * (yr * up + xr * right);
        Rd = cameraCtrl.getFar() * (Rf - Ro).normalized();
    }

    // trace!
    RaycastResult result = rt->raycast(Ro, Rd, false);

    // if we hit something, fetch a color and insert into image
    Vec3f Ei(0.0f);

    if (result.tri != nullptr) {
        if (debugVis) {
            PathVisualizationNode node;
            node.labels.push_back(PathVisualizationLabel("start", result.orig));  // start label for eye pos
            node.lines.push_back(PathVisualizationLine(result.orig, result.point, Vec3f(1.0f, 1.0f, 1.0f)));  // white line
            node.lines.push_back(PathVisualizationLine(result.point, result.point + result.tri->normal() * .1f, Vec3f(1.0f, 0.0f, 0.0f)));
            node.labels.push_back(PathVisualizationLabel("direct", result.point));
            visualization.push_back(node);
        }

        // blinn-phong
        float gloss = result.tri->m_material->glossiness;

        float normalization = (gloss + 2.0f) / (4.0f * FW_PI * (2.0f - pow(2.0f, -gloss / 2.0f)));

        int bounce = 0;
        int bounces = abs(ctx.m_bounces);  // always do #m_bounces bounces

        Vec3f throughput(1, 1, 1), color(0.0f);

        // float p = 1.0f;

        // direct light:

        Vec3f O = result.point;           // hit position
        Vec3f V = normalize(result.dir);  // Ray direction

        // get surface parameters
        Vec3f diffuse, N, specular;
        getTextureParameters(result, diffuse, N, specular);

        int light_index = getLightToSample(O, lights, R);

        // sample light source
        float pdf;  // probability density function
        Vec3f Pl;   // point
        (*lights)[light_index].sample(pdf, Pl, samplerBase, bounce, R);

        Vec3f L = Pl - O;          // vec to light
        Vec3f D = L.normalized();  // dir to light

        Vec3f emission = (*lights)[light_index].getEmission() * m_intensity;

        // shadow ray
        if (!rt->raycast(O + 0.0001f * N, L, true)) {
            // check for backface
            if (dot(N, V) > 0.0f) {
                N *= -1.0f;
            }

            // area light normal
            Vec3f Nl = (*lights)[light_index].getNormal();

            // theta = angle between vertex normal and dir to light
            F32 cos_theta = max(0.0f, dot(N, D));

            // theta_l = angle between the vector yx (from light to vertex) and the surface normal of the light.
            F32 cos_theta_l = max(0.0f, dot(Nl, -D));

            // distance to light source
            float d = L.length();

            float inv_d = min(1.0f / (d * d), 100000.0f);  // avoid infinities
            float inv_pdf = min(1.0f / pdf, 100000.0f);

            // halfway vector
            Vec3f H = normalize(D - V);

            // diffuse color
            color = diffuse;

            // specular color: Blinn-Phong with normalization
            color += specular * pow(max(0.0f, dot(N, H)), gloss) * normalization;

            // accumulate
            Ei += color * invPi * emission * cos_theta * cos_theta_l * inv_d * inv_pdf;  // * throughput / p
        }

        // At this point you should determine if the path should be terminated. There
        // are two alternative modes: a fixed number of bounces, or Russian roulette
        // started after a given number of bounces.The mode and the numbers are set
        // by the user, and passed to the function in the variable ctx.m bounces. The
        // variable has a negative value if Russian roulette is enabled, and a positive
        // value if not. So for example if the value is -3, this means that the first 3
        // iterations will always cast continuation rays, but after that the loop will be
        // terminated with some probability on each iteration, with the ray contribution
        // compensated accordingly.

        // bounces
        while (bounce < bounces) {
            // Draw a cosine weighted direction
            V = normalize(formBasis(N) * drawCosineWeightedDirection(samplerBase, bounce, R));

            // p *= dot(N, V) * invPi; // cos(a) / pi

            throughput *= diffuse;  // * invPi * dot(N, V); // BRDF * cos(a) = kd / pi * cos(a)

            // trace ray
            result = rt->raycast(O + 0.0001f * N, cameraCtrl.getFar() * V, false);

            if (!result) {
                break;
            }

            O = result.point;

            getTextureParameters(result, diffuse, N, specular);

            light_index = getLightToSample(O, lights, R);

            (*lights)[light_index].sample(pdf, Pl, samplerBase, bounce, R);

            L = Pl - O;
            D = L.normalized();

            // shadow ray
            if (!rt->raycast(O + 0.0001f * N, L, true)) {
                // check for backface
                if (dot(N, V) > 0.0f) {
                    N *= -1.0f;
                }

                // area light normal
                Vec3f Nl = (*lights)[light_index].getNormal();

                // theta = angle between vertex normal and dir to light
                F32 cos_theta = max(0.0f, dot(N, D));

                // theta_l = angle between the vector yx (from light to vertex) and the surface normal of the light.
                F32 cos_theta_l = max(0.0f, dot(Nl, -D));

                // distance to light source
                float d = L.length();

                // float inv_p = min(1.0f / p, 10000000.0f);

                float inv_d = min(1.0f / (d * d), 100000.0f);
                float inv_pdf = min(1.0f / pdf, 100000.0f);

                // halfway vector
                Vec3f H = normalize(D - V);

                // diffuse color
                color = diffuse;

                // specular color: Blinn-Phong with normalization
                color += specular * pow(max(0.0f, dot(N, H)), gloss) * normalization;

                // accumulate
                Ei += color * invPi * throughput * emission * cos_theta * cos_theta_l * inv_d * inv_pdf;  // * inv_p
            }
            bounce += 1;

            if (debugVis) {
                // show debug path in yellow for indirect
                PathVisualizationNode node;
                node.lines.push_back(PathVisualizationLine(result.orig, result.point, Vec3f(1.0f, 1.0f, 0.0f)));
                node.lines.push_back(
                    PathVisualizationLine(result.point, result.point + result.tri->normal() * .1f, Vec3f(1.0f, 0.0f, 0.0f)));
                node.labels.push_back(PathVisualizationLabel("bounce " + std::to_string(bounce), result.point));

                visualization.push_back(node);
            }
        }

        // russian roulette
        if (ctx.m_bounces < 0) {
            float terminate = R.getF32(0.0f, 1.0f);
            float inv_prob = 1.0f / m_terminationProbability;

            // continue path or terminate?
            while (terminate > m_terminationProbability) {
                // Draw a cosine weighted direction
                V = normalize(formBasis(N) * drawCosineWeightedDirection(samplerBase, bounce, R));

                // p *= dot(N, V) * invPi; // cos(a) / pi

                // throughput *= diffuse * invPi * dot(N, V); // BRDF * cos(a) = kd / pi * cos(a)

                throughput *= diffuse;

                // trace ray
                result = rt->raycast(O + 0.0001f * N, cameraCtrl.getFar() * V, false);

                if (!result) {
                    break;
                }

                O = result.point;

                getTextureParameters(result, diffuse, N, specular);

                light_index = getLightToSample(O, lights, R);

                (*lights)[light_index].sample(pdf, Pl, samplerBase, bounce, R);

                L = Pl - O;
                D = L.normalized();

                // shadow ray
                if (!rt->raycast(O + 0.0001f * N, L, true)) {
                    // check for backface
                    if (dot(N, V) > 0.0f) {
                        N *= -1.0f;
                    }

                    // area light normal
                    Vec3f Nl = (*lights)[light_index].getNormal();

                    // theta = angle between vertex normal and dir to light
                    F32 cos_theta = max(0.0f, dot(N, D));

                    // theta_l = angle between the vector yx (from light to vertex) and the surface normal of the light.
                    F32 cos_theta_l = max(0.0f, dot(Nl, -D));

                    // length to light source
                    float d = L.length();

                    // float inv_p = min(1.0f / p, 10000000.0f);d

                    float inv_d = min(1.0f / (d * d), 100000.0f);
                    float inv_pdf = min(1.0f / pdf, 100000.0f);

                    // halfway vector
                    Vec3f H = normalize(D - V);

                    // diffuse color
                    color = diffuse;

                    // specular color: Blinn-Phong with normalization
                    color += specular * pow(max(0.0f, dot(N, H)), gloss) * normalization;

                    // accumulate
                    Ei += color * invPi * throughput * emission * cos_theta * cos_theta_l * inv_d * inv_pdf * inv_prob;
                }
                terminate = R.getF32(0.0f, 1.0f);
                bounce += 1;

                if (debugVis) {
                    // show roulette bounces in green
                    PathVisualizationNode node;
                    node.lines.push_back(PathVisualizationLine(result.orig, result.point, Vec3f(0.0f, 1.0f, 0.0f)));
                    node.lines.push_back(
                        PathVisualizationLine(result.point, result.point + result.tri->normal() * .1f, Vec3f(1, 0, 0)));
                    node.labels.push_back(PathVisualizationLabel("roulette" + std::to_string(bounce), result.point));
                    visualization.push_back(node);
                }
            }
        }
    }
    return Ei;
}

Vec3f PathTraceRenderer::traceWhitted(float x, float y, PathTracerContext& ctx, int samplerBase, Random& R, std::vector<PathVisualizationNode>& visualization) {
    // const MeshWithColors* scene = ctx.m_scene;
    RayTracer* rt = ctx.m_rt;
    Image* image = ctx.m_image.get();
    const CameraControls& cameraCtrl = *ctx.m_camera;
    std::vector<AreaLight>* lights = ctx.m_lights;

    // get camera orientation and projection
    Mat4f worldToCamera = cameraCtrl.getWorldToCamera();
    Mat4f projection = Mat4f::fitToView(Vec2f(-1, -1), Vec2f(2, 2), image->getSize()) * cameraCtrl.getCameraToClip();

    // inverse projection from clip space to world space
    Mat4f invP = (projection * worldToCamera).inverted();

    // point on front plane in homogeneous coordinates
    Vec4f P0(x, y, 0.0f, 1.0f);
    // point on back plane in homogeneous coordinates
    Vec4f P1(x, y, 1.0f, 1.0f);

    // apply inverse projection, divide by w to get object-space points
    Vec4f Roh = (invP * P0);
    Vec3f Ro = (Roh * (1.0f / Roh.w)).getXYZ();
    Vec4f Rdh = (invP * P1);
    Vec3f Rd = (Rdh * (1.0f / Rdh.w)).getXYZ();

    // Subtract front plane point from back plane point,
    // yields ray direction.
    // NOTE that it's not normalized; the direction Rd is defined
    // so that the segment to be traced is [Ro, Ro+Rd], i.e.,
    // intersections that come _after_ the point Ro+Rd are to be discarded.
    Rd = Rd - Ro;

    if (m_depthOfField) {
        // focal plane
        Vec3f Rn = Rd.normalized();                // normalized ray direction
        Vec3f Pn = cameraCtrl.getForward();        // "focal plane" normal
        float Pd = m_focalDistance / dot(Rn, Pn);  // distance from ray origin to focal plane:
        Vec3f Rf = Ro + Pd * Rn;                   // origin + distance * direction

        // random sample on disk (aperture)
        F32 xr, yr, a, b;
        a = R.getF32(-1.0, 1.0f);
        b = R.getF32(-1.0, 1.0f);

        mapToDisk(a, b, xr, yr);

        // camera orientation
        Vec3f up = cameraCtrl.getOrientation().getCol(0);
        Vec3f right = cameraCtrl.getOrientation().getCol(1);

        // shift ray origin and calculate new direction
        Ro += m_aperture * (yr * up + xr * right);
        Rd = cameraCtrl.getFar() * (Rf - Ro).normalized();
    }

    // trace!
    RaycastResult result = rt->raycast(Ro, Rd, false);

    // if we hit something, fetch a color and insert into image
    Vec3f color(0.0f);

    if (result.tri != nullptr) {
        Vec3f O = result.point;           // hit position
        Vec3f V = normalize(result.dir);  // Ray direction

        Vec3f diffuse, N, specular;
        getTextureParameters(result, diffuse, N, specular);

        // blinn-phong
        F32 gloss = result.tri->m_material->glossiness;
        F32 normalization = (gloss + 2.0f) / (4.0f * FW_PI * (2.0f - pow(2.0f, -gloss / 2.0f)));

        int light_index = getLightToSample(O, lights, R);

        // draw #m_numLightRays samples from area light and check visibility
        float pdf, shadow = 0.0f, d;
        Vec3f Pl, L, D;
        for (int i = 0; i < m_numLightRays; i++) {
            (*lights)[light_index].sampleDisk(pdf, Pl, samplerBase, 0, R);
            L = Pl - O;
            D = L.normalized();
            if (!rt->raycast(O + 0.0001f * D, L, true)) {
                shadow += 1.0f;
            }
        }
        L = (*lights)[light_index].getPosition() - O;  // pos to light
        D = L.normalized();                            // Light direction
        d = L.length();                                // distance to light

        F32 attenuation = m_intensity / (m_attenuation * pow(d, 2.0f) + m_attenuation * d + 0.5f);

        // halfway vector
        Vec3f H = normalize(D - V);

        // diffuse color
        color = diffuse * max(0.0f, dot(N, D));

        // specular color: Blinn-Phong with normalization
        color += specular * pow(max(0.0f, dot(N, H)), gloss) * normalization;

        // distance attenuation
        color *= attenuation;

        // shadow factor
        color *= shadow / m_numLightRays;
    }
    return color;
}

// This function is responsible for asynchronously generating paths for a given block.
void PathTraceRenderer::pathTraceBlock(MulticoreLauncher::Task& t) {
    PathTracerContext& ctx = *(PathTracerContext*)t.data;

    // const MeshWithColors* scene       = ctx.m_scene;
    // RayTracer* rt                     = ctx.m_rt;
    Image* image = ctx.m_image.get();
    const CameraControls& cameraCtrl = *ctx.m_camera;
    // std::vector<AreaLight>* lights    = ctx.m_lights;

    // make sure we're on CPU
    image->getMutablePtr();

    // get camera orientation and projection
    Mat4f worldToCamera = cameraCtrl.getWorldToCamera();
    Mat4f projection = Mat4f::fitToView(Vec2f(-1, -1), Vec2f(2, 2), image->getSize()) * cameraCtrl.getCameraToClip();

    // inverse projection from clip space to world space
    Mat4f invP = (projection * worldToCamera).inverted();

    // get the block which we are rendering
    PathTracerBlock& block = ctx.m_blocks[t.idx];

    // Not used but must be passed to tracePath
    std::vector<PathVisualizationNode> dummyVisualization;

    // this is bogus, just to make the random numbers change on each iteration
    static std::atomic<uint32_t> seed = 0;
    uint32_t current_seed = seed.fetch_add(1);
    Random R(t.idx + current_seed);

    // Sobol sequence base
    int base = t.idx * R.getS32(1, 9) + R.getS32(1, 12345);

    int height = image->getSize().y;
    int width = image->getSize().x;

    float div_x = 1.0f / (float)width;
    float div_y = 1.0f / (float)height;

    float filt_scale = 2.0f / m_filterWidth;

    float a = 0.0f, b = 0.0f;

    if (m_sobolBlock) {
        float scaleToAaRays = 1.0f / (float)m_NumAaRays;
        for (int i = 0; i < block.m_width * block.m_height * m_NumAaRays; ++i) {
            if (ctx.m_bForceExit) {
                return;
            }

            // ray origin
            float px = block.m_x + block.m_width * sobol::sample(base + i, 0);
            float py = block.m_y + block.m_height * sobol::sample(base + i, 1);
            float x = px * div_x * 2.0f - 1.0f;
            float y = py * div_y * -2.0f + 1.0f;

            Vec3f color;
            if (m_whitted) {
                color = traceWhitted(x, y, ctx, base + i, R, dummyVisualization);
            } else {
                color = tracePath(x, y, ctx, base + i, R, dummyVisualization);
            }

            // get pixel
            int pixel_x = min((int)floor(px), width - 1);
            int pixel_y = min((int)floor(py), height - 1);

            Vec4f prev = image->getVec4f(Vec2i(pixel_x, pixel_y));
            prev += Vec4f(color, 1.0f) * scaleToAaRays;
            image->setVec4f(Vec2i(pixel_x, pixel_y), prev);
        }
    } else {
        for (int i = 0; i < block.m_width * block.m_height; ++i) {
            if (ctx.m_bForceExit) {
                return;
            }

            int pixel_x = block.m_x + (i % block.m_width);
            int pixel_y = block.m_y + (i / block.m_width);

            // AA: draw #m_NumAaRays samples around pixel
            Vec4f color_AA(0.0f);
            for (int k = 0; k < m_NumAaRays; k++) {
                // int r = R.getU32(1, 8765);
                a = m_filterWidth * (sobol::sample(base + i + k, 0) - 0.5f) + 0.5f;
                b = m_filterWidth * (sobol::sample(base + i + k, 1) - 0.5f) + 0.5f;

                // generate ray through pixel
                float px = (float)pixel_x + a;
                float py = (float)pixel_y + b;
                if (px < 0.0f) {
                    px *= -1.0f;
                }
                if (py < 0.0f) {
                    py *= -1.0f;
                }
                if (px > width) {
                    px -= a;
                }
                if (py > height) {
                    py -= b;
                }
                float x = px * div_x * 2.0f - 1.0f;
                float y = py * div_y * -2.0f + 1.0f;

                // trace
                Vec3f color;
                if (m_whitted) {
                    color = traceWhitted(x, y, ctx, base + i, R, dummyVisualization);
                } else {
                    color = tracePath(x, y, ctx, base + i, R, dummyVisualization);
                }

                // filter
                color_AA += Gauss(filt_scale * (a - 0.5f), filt_scale * (b - 0.5f)) * Vec4f(color, 1.0f);
            }
            // normalize with accumulated filter weight w
            color_AA /= color_AA.w;

            color_AA = saturate(color_AA, m_attenuation);

            // put pixel to image
            Vec4f prev = image->getVec4f(Vec2i(pixel_x, pixel_y));
            prev += color_AA;
            image->setVec4f(Vec2i(pixel_x, pixel_y), prev);
        }
    }
}

void PathTraceRenderer::startPathTracingProcess(
    const MeshWithColors* scene,
    std::vector<AreaLight>* lights,
    RayTracer* rt,
    Image* dest,
    int bounces,
    const CameraControls& camera) {
    FW_ASSERT(!m_context.m_bForceExit);

    m_context.m_bForceExit = false;
    m_context.m_bResidual = false;
    m_context.m_camera = &camera;
    m_context.m_rt = rt;
    m_context.m_scene = scene;
    m_context.m_lights = lights;
    m_context.m_pass = 0;
    m_context.m_bounces = bounces;
    m_context.m_image.reset(new Image(dest->getSize(), ImageFormat::RGBA_Vec4f));
    m_context.m_destImage = dest;
    m_context.m_image->clear();

    // add rendering blocks
    m_context.m_blocks.clear();
    {
        int block_size = 32;
        int image_width = dest->getSize().x;
        int image_height = dest->getSize().y;
        int block_count_x = (image_width + block_size - 1) / block_size;
        int block_count_y = (image_height + block_size - 1) / block_size;

        for (int y = 0; y < block_count_y; ++y) {
            int block_start_y = y * block_size;
            int block_end_y = FW::min(block_start_y + block_size, image_height);
            int block_height = block_end_y - block_start_y;

            for (int x = 0; x < block_count_x; ++x) {
                int block_start_x = x * block_size;
                int block_end_x = FW::min(block_start_x + block_size, image_width);
                int block_width = block_end_x - block_start_x;

                PathTracerBlock block;
                block.m_x = block_size * x;
                block.m_y = block_size * y;
                block.m_width = block_width;
                block.m_height = block_height;

                m_context.m_blocks.push_back(block);
            }
        }
    }

    dest->clear();

    m_context.start = std::chrono::steady_clock::now();

    m_launcher.setNumThreads(m_launcher.getNumCores());
    // DEBUG: Enable this for debugging theading issues
    // m_launcher.setNumThreads(1);

    m_launcher.popAll();
    m_launcher.push(pathTraceBlock, &m_context, 0, (int)m_context.m_blocks.size());
}

void PathTraceRenderer::updatePicture(Image* dest) {
    FW_ASSERT(m_context.m_image != 0);
    FW_ASSERT(m_context.m_image->getSize() == dest->getSize());

    for (int i = 0; i < dest->getSize().y; ++i) {
        for (int j = 0; j < dest->getSize().x; ++j) {
            Vec4f D = m_context.m_image->getVec4f(Vec2i(j, i));
            if (D.w != 0.0f) {
                D = D * (1.0f / D.w);
            }
            // Gamma correction.
            Vec4f color = Vec4f(FW::pow(D.x, 1.0f / 2.2f), FW::pow(D.y, 1.0f / 2.2f), FW::pow(D.z, 1.0f / 2.2f), D.w);

            dest->setVec4f(Vec2i(j, i), color);
        }
    }
}

void PathTraceRenderer::checkFinish() {
    // Print progress
    printf(" %.2f%% done     \r", 100.0f * m_launcher.getNumFinished() / m_launcher.getNumTasks());

    // have all the vertices from current bounce finished computing?
    if (m_launcher.getNumTasks() == m_launcher.getNumFinished()) {
        // yes, remove from task list
        m_launcher.popAll();

        ++m_context.m_pass;

        float time = static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_context.start).count()) * 0.001f;

        // calculate average rays per second
        m_TotalRays += m_context.m_rt->getRayCount();
        m_raysPerSecond = m_TotalRays / time;
        m_context.m_rt->resetRayCounter();

        // print stats
        if (time < 60.0f) {
            std::printf("  Done! time: %5.2f s   ", time);
        } else {
            std::printf("  Done! time: %5.2f min   ", time / 60.0f);
        }
        std::printf("Total Mrays: %6.1f (%5.2f Mrays/sec)\n", m_TotalRays / 1000000.0f, m_raysPerSecond / 1000000.0f);

        // after the completion of each full round through the image
        String fn = sprintf("pt-%03dppp.png", m_context.m_pass);
        File outfile(fn, File::Create);
        exportLodePngImage(outfile, m_context.m_destImage);

        if (!m_context.m_bForceExit) {
            // keep going
            m_launcher.setNumThreads(m_launcher.getNumCores());
            // DEBUG: Enable this for debugging theading issues
            // m_launcher.setNumThreads(1);
            m_launcher.popAll();
            m_launcher.push(pathTraceBlock, &m_context, 0, (int)m_context.m_blocks.size());
            ::printf("Pass %d\n", m_context.m_pass);
        } else {
            ::printf("\nStopped!\n");
        }
    }
}

void PathTraceRenderer::stop() {
    m_context.m_bForceExit = true;

    if (isRunning()) {
        m_context.m_bForceExit = true;
        while (m_launcher.getNumTasks() > m_launcher.getNumFinished()) {
            Sleep(1);
        }
        m_launcher.popAll();
    }

    m_context.m_bForceExit = false;
}

}  // namespace FW
