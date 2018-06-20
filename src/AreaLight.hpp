#pragma once

#include <base/Math.hpp>
#include <base/Random.hpp>
#include <io/StateDump.hpp>

#include "sobol.h"

namespace FW
{

class GLContext;

// a simple square-shaped area light source

class AreaLight
{
public:
    AreaLight() : m_E(100.0f, 100.0f, 100.0f), m_size(0.25f,0.25f) 
	{
		this->setPosition(Vec3f(0.0f));
		this->setOrientation(Mat3f());
	}

	AreaLight(Vec3f E, Vec2f size, float intensity) : m_E(E), m_size(size), m_intensity(intensity) 
	{ 
		this->setPosition(Vec3f(0.0f));
		this->setOrientation(Mat3f());
		this->updatePower(); 
	}

    // this function draws samples on the light source for computing direct illumination
    // the "base" input is used for driving QMC samplers
    void			sample     (float& pdf, Vec3f& p, int base, int dimension, Random& rnd );
	void			sample_disk(float& pdf, Vec3f& p, int base, int dimension, Random& rnd);

    Vec3f			getPosition(void) const			{ return Vec4f(m_xform.getCol(3)).getXYZ(); }
    void			setPosition(const Vec3f& p)		{ m_xform.setCol(3, Vec4f(p, 1.0f)); }

    Mat3f			getOrientation(void) const		{ return m_xform.getXYZ(); }
    void			setOrientation(const Mat3f& R)	{ m_xform.setCol(0,Vec4f(R.getCol(0),0.0f)); m_xform.setCol(1,Vec4f(R.getCol(1),0.0f)); m_xform.setCol(2,Vec4f(R.getCol(2),0.0f)); }

    Vec3f			getNormal(void) const			{ return -Vec4f(m_xform.getCol(2)).getXYZ(); }

    Vec2f			getSize(void) const				{ return m_size; }
    void			setSize(const Vec2f& s)			{ m_size = s; }

	float			getArea(void) const { return 4.0f * m_size.x * m_size.y; }

    Vec3f			getEmission(void) const			{ return m_E; }
    void			setEmission(const Vec3f& E)		{ m_E = E; }

	float			getIntensity(void) const		{ return m_intensity; }
	void			setIntensity(const float i)		{ m_intensity = i; }

	float			getPower(void) const { return m_power; }
	void			updatePower() { m_power = 4.0f * m_size.x * m_size.y * m_E.length() * m_intensity; } // emissive power scaled with intensity

    void			draw( const Mat4f& worldToCamera, const Mat4f& projection, bool whitted);

    void			readState( StateDump& d )			{ d.pushOwner("areaLight"); d.get(m_xform,"xform"); d.get(m_size,"size"); d.get(m_E,"E"); d.popOwner(); }
    void			writeState( StateDump& d ) const	{ d.pushOwner("areaLight"); d.set(m_xform,"xform"); d.set(m_size,"size"); d.set(m_E,"E"); d.popOwner(); }

protected:
    Mat4f	m_xform;	// Encodes position and orientation in world space.
    Vec2f	m_size;		// Physical size of the emitter from the center of the light, i.e. half of the total width/height.
    Vec3f	m_E;		// Diffuse emission (W/m^2).
	float	m_intensity;
	float	m_power;
};

} // namespace FW
