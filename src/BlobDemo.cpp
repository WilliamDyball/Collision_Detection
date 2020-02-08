/*
 * The Blob demo.
 *
 */
#include <gl/glut.h>
#include "app.h"
#include "coreMath.h"
#include "pcontacts.h"
#include "pworld.h"
#include <stdio.h>
#include <cassert>
#include <iostream>
using namespace std;

#define BLOB_COUNT 8
#define PLATFORM_COUNT 7

const Vector2 Vector2::GRAVITY = Vector2(0, -9.81);

/**
 * Platforms are two dimensional: lines on which the
 * particles can rest. Platforms are also contact generators for the physics.
 */

class Platform : public ParticleContactGenerator
{
public:
	Vector2 start;
	Vector2 end;
	float restitution;
	/**
	 * Holds a pointer to the particles we're checking for collisions with.
	 */
	Particle *particle;

	virtual unsigned addContact(ParticleContact *contact, unsigned limit) const;
};

unsigned Platform::addContact(ParticleContact *contact, unsigned limit) const
{

	static float restitution = 1.0f;
	unsigned used = 0;

	for (unsigned i = 0; i < BLOB_COUNT; i++)
	{
		if (used >= limit) return used;

		// Check for penetration
		Vector2 toParticle = particle[i].getPosition() - start;
		Vector2 lineDirection = end - start;

		float projected = toParticle * lineDirection;
		float platformSqLength = lineDirection.squareMagnitude();
		float squareRadius = particle[i].getRadius()*particle[i].getRadius();;

		if (projected <= 0)
		{
			// The blob is nearest to the start point
			if (toParticle.squareMagnitude() < squareRadius)
			{
				// We have a collision
				contact->contactNormal = toParticle.unit();
				contact->restitution = restitution;
				contact->particle[0] = particle + i;
				contact->particle[1] = 0;
				contact->penetration = particle[i].getRadius() - toParticle.magnitude();
				used++;
				contact++;
			}
		}
		else if (projected >= platformSqLength)
		{
			// The blob is nearest to the end point
			toParticle = particle[i].getPosition() - end;
			if (toParticle.squareMagnitude() < squareRadius)
			{
				// We have a collision
				contact->contactNormal = toParticle.unit();
				contact->restitution = restitution;
				contact->particle[0] = particle + i;
				contact->particle[1] = 0;
				contact->penetration = particle[i].getRadius() - toParticle.magnitude();
				used++;
				contact++;
			}
		}
		else
		{
			// the blob is nearest to the middle.
			float distanceToPlatform = toParticle.squareMagnitude() - projected*projected / platformSqLength;
			if (distanceToPlatform < squareRadius)
			{
				// We have a collision
				Vector2 closestPoint = start + lineDirection*(projected / platformSqLength);

				contact->contactNormal = (particle[i].getPosition() - closestPoint).unit();
				contact->restitution = restitution;
				contact->particle[0] = particle + i;
				contact->particle[1] = 0;
				contact->penetration = particle[i].getRadius() - sqrt(distanceToPlatform);
				used++;
				contact++;
			}
		}
	}
	return used;
}

class ParticleCollision : public ParticleContactGenerator
{
public:
	Particle *particle;

	virtual unsigned addContact(ParticleContact *contact, unsigned limit) const;
};

unsigned ParticleCollision::addContact(ParticleContact *contact, unsigned limit) const
{
	unsigned used = 0;

	for (unsigned i = 0; i < BLOB_COUNT; i++)
	{
		for (unsigned j = i + 1; j < BLOB_COUNT; j++)
		{
			/*if (i == j)
				break;*/

			if (used >= limit)
				return used;

			Vector2 toParticle = particle[j].getPosition() - particle[i].getPosition();

			float radius1 = particle[i].getRadius();
			float radius2 = particle[j].getRadius();

			float distanceLength = toParticle.magnitude();

			if (distanceLength < (radius1 + radius2))
			{
				contact->contactNormal = toParticle.unit();
				contact->particle[0] = particle + j;
				contact->particle[1] = particle + i;
				contact->penetration = ((particle[i].getRadius() + particle[j].getRadius()) - toParticle.magnitude());
				used++;
				contact++;
				//cout << "contact" << endl;
			}
		}
	}
	if (used > 1)
		cout << used << endl;
	return used;
}

class BlobDemo : public Application
{
	Particle *blob;

	Platform *platforms;

	ParticleCollision *blobCol;

	ParticleWorld world;

public:
	/** Creates a new demo object. */
	BlobDemo();
	virtual ~BlobDemo();

	/** Returns the window title for the demo. */
	virtual const char* getTitle();

	/** Display the particles. */
	virtual void display();

	/** Update the particle positions. */
	virtual void update();

};

// Method definitions
BlobDemo::BlobDemo() :

world(PLATFORM_COUNT + BLOB_COUNT + BLOB_COUNT, PLATFORM_COUNT)
{
	width = 500; height = 500;
	nRange = 100.0;

	float margin = 0.95;

	// Create the blob storage
	blob = new Particle[BLOB_COUNT];

	// Create blob collision
	blobCol = new ParticleCollision[BLOB_COUNT];

	// Create the platforms
	platforms = new Platform[PLATFORM_COUNT];

#pragma region Platform positions
	platforms[0].start = Vector2(-50.0, 0.0);
	platforms[0].end = Vector2(20.0, -10.0);
	platforms[0].restitution = 0.6;

	platforms[1].start = Vector2(-nRange*margin, -nRange*margin);
	platforms[1].end = Vector2(nRange*margin, -nRange*margin);
	platforms[1].restitution = 1.0;

	platforms[2].start = Vector2(-nRange*margin, nRange*margin);
	platforms[2].end = Vector2(nRange*margin, nRange*margin);
	platforms[2].restitution = 1.0;

	platforms[3].start = Vector2(-nRange*margin, -nRange*margin);
	platforms[3].end = Vector2(-nRange*margin, nRange*margin);
	platforms[3].restitution = 1.0;

	platforms[4].start = Vector2(nRange*margin, -nRange*margin);
	platforms[4].end = Vector2(nRange*margin, nRange*margin);
	platforms[4].restitution = 1.0;

	platforms[5].start = Vector2(80.0, -40.0);
	platforms[5].end = Vector2(0.0, -70.0);
	platforms[5].restitution = 0.6;

	platforms[6].start = Vector2(-20.0, -80.0);
	platforms[6].end = Vector2(-80.0, -50.0);
	platforms[6].restitution = 0.4;
#pragma endregion


	// Make sure the platform knows which particle it should collide with.
	for (unsigned i = 0; i < PLATFORM_COUNT; i++)
	{
		// Make sure the platform knows which particles it 
		// should collide with.
		platforms[i].particle = blob;
		world.getContactGenerators().push_back(platforms + i);
	}

	// Create the blobs
	Platform *p = platforms;
	Vector2 delta = p->end - p->start;
	for (unsigned i = 0; i < BLOB_COUNT; i++)
	{
		blob[i].setPosition(-90.0 + i * 20, 90.0);
		//blob[i].setPosition(20.0 + i * 10, 90.0);
		blob[i].setRadius(3 + i);
		blob[i].setVelocity(0, 0);
		blob[i].setDamping(0.9);
		blob[i].setAcceleration(Vector2::GRAVITY * 5.0f * (i + 1));
		blob[i].setMass(3.0f * i + 5);
		blob[i].clearAccumulator();

		world.getParticles().push_back(blob + i);
	}

	for (unsigned i = 0; i < BLOB_COUNT; i++)
	{
		// Make sure the platform knows which particles it 
		// should collide with.
		blobCol[i].particle = blob;
		world.getContactGenerators().push_back(blobCol + i);
	}

}

BlobDemo::~BlobDemo()
{
	delete blob;
}

void BlobDemo::display()
{
	Application::display();

	glBegin(GL_LINES);
	glColor3f(0, 1, 1);
	for (unsigned i = 0; i < PLATFORM_COUNT; i++)
	{
		const Vector2 &p0 = platforms[i].start;
		const Vector2 &p1 = platforms[i].end;
		glVertex2f(p0.x, p0.y);
		glVertex2f(p1.x, p1.y);
	}

	glEnd();

	for (unsigned i = 0; i < BLOB_COUNT; i++)
	{
		glColor3f((i % 2) ? 0 : 1, (i % 2) ? 1 : 0, 0);
		const Vector2 &p = blob[i].getPosition();
		glPushMatrix();
		glTranslatef(p.x, p.y, 0);
		glutSolidSphere(blob[i].getRadius(), 12, 12);
		glPopMatrix();

		Vector2 temp = blob[i].getVelocity();

		//cout << "blob" << i << ": " << temp.x << temp.y << endl;
	}

	glutSwapBuffers();

}

void BlobDemo::update()
{
	// Recenter the axes
	float duration = timeinterval / 1000;
	// Run the simulation
	world.runPhysics(duration);

	Application::update();
}

const char* BlobDemo::getTitle()
{
	return "Blob Demo";
}

/**
 * Called by the common demo framework to create an application
 * object (with new) and return a pointer.
 */
Application* getApplication()
{
	return new BlobDemo();
}