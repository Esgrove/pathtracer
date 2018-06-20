#include "AreaLight.hpp"
#include "util.hpp"

namespace FW {

void AreaLight::draw(const Mat4f& worldToCamera, const Mat4f& projection, bool whitted) {
    glUseProgram(0);
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf((float*)&projection);
    glMatrixMode(GL_MODELVIEW);

	if (whitted) {

		// plot light as a sphere
		// https://en.wikipedia.org/wiki/Spherical_coordinate_system

		Mat4f M = worldToCamera * m_xform;
		glLoadMatrixf((float*)&M);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glColor4f(m_E.x / 100.0f, m_E.y / 100.0f, m_E.z / 100.0f, 0.6f);

		int num_segments = 32;

        for (int i = 0; i < num_segments; i++) {

            float theta1 = FW_PI * (float)(i)   / (float)num_segments;
            float theta2 = FW_PI * (float)(i+1) / (float)num_segments;
   
            glBegin(GL_QUAD_STRIP);

            for (int j = 0; j <= num_segments; j++) {

                float phi = 2.0f * FW_PI * (float)j / (float)num_segments;

                float x1 = m_size.x * sin(theta1) * cos(phi);
                float y1 = m_size.y * sin(theta1) * sin(phi);
				float z1 = m_size.y * cos(theta1);

				float x2 = m_size.x * sin(theta2) * cos(phi);
				float y2 = m_size.y * sin(theta2) * sin(phi);
				float z2 = m_size.y * cos(theta2);
   
                glVertex3f(x1, y1, z1);
				glVertex3f(x2, y2, z2);
            }
            glEnd();
        }
		glDisable(GL_BLEND);
	}
	else {
		Mat4f S = Mat4f::scale(Vec3f(m_size, 1));
		Mat4f M = worldToCamera * m_xform * S;
		glLoadMatrixf((float*)&M);

		glBegin(GL_TRIANGLES);
		glColor3f(m_E.x / 100.0f, m_E.y / 100.0f, m_E.z / 100.0f);
		glVertex3f(1,1,0); glVertex3f(1,-1,0); glVertex3f( -1,-1,0 );
		glVertex3f(1,1,0); glVertex3f( -1,-1,0 ); glVertex3f(-1,1,0); 
		glEnd();
	}
}

void AreaLight::sample(float& pdf, Vec3f& p, int base, int dimension, Random& rnd) 
{
	// orientation
	Vec3f tangent   = Vec4f(m_xform.getCol(0)).getXYZ();
	Vec3f bitangent = Vec4f(m_xform.getCol(1)).getXYZ();

	// Sobol
	int dim = 4 + 4 * dimension; // dimensions 4 & 5 
	int r = rnd.getS32(1, 10000);
	float x = (2.0f * sobol::sample(base + r, dim    ) - 1.0f) * m_size.x;
	float y = (2.0f * sobol::sample(base + r, dim + 1) - 1.0f) * m_size.y;

	// point on area
	p = Vec4f(m_xform.getCol(3)).getXYZ() + tangent * x + bitangent * y;

	// probability density function = 1 / area
	pdf = 1.0f / (4.0f * m_size.x * m_size.y);
}

void AreaLight::sample_disk(float& pdf, Vec3f& p, int base, int dimension, Random& rnd) 
{
	// orientation
	Vec3f tangent   = Vec4f(m_xform.getCol(0)).getXYZ();
	Vec3f bitangent = Vec4f(m_xform.getCol(1)).getXYZ();

	// Sobol
	int dim = 4 + 4 * dimension;
	int rand = rnd.getS32(1, 1000);

	float x, y, a, b;

	a = (2.0f * sobol::sample(base + rand, dim    ) - 1.0f) * m_size.x;
	b = (2.0f * sobol::sample(base + rand, dim + 1) - 1.0f) * m_size.y;

	mapToDisk(a, b, x, y);

	// point on area
	p = Vec4f(m_xform.getCol(3)).getXYZ() + tangent * x + bitangent * y;

	// probability density function = 1 / circle area
	pdf = 1.0f / (FW_PI * m_size.x * m_size.y);
}

} // namespace FW
