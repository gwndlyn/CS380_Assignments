#include <pch.h>
#include "Projects/ProjectOne.h"
#include "Agent/CameraAgent.h"

void ProjectOne::setup()
{
	// Create your inital agents
	//agents->create_behavior_agent("ExampleAgent", BehaviorTreeTypes::Example);
	
	//colors: bird - red, yellow - dog, purple - owner, blue - customer

	auto bird = agents->create_behavior_agent("BirdAgent", BehaviorTreeTypes::Bird);
	bird->set_position(Vec3(10.0f, 0.0f, 10.0f));
	bird->set_color(Vec3(1.0f, 0.1f, 0.1f));
	bird->set_movement_speed(50.0f);

	auto customer = agents->create_behavior_agent("CustomerAgent", BehaviorTreeTypes::Customer);
	customer->set_position(Vec3(90.0f, 0.0f, 90.0f));
	customer->set_color(Vec3(0.0f, 1.0f, 1.0f));
	customer->set_movement_speed(20.0f);

	auto dog = agents->create_behavior_agent("DogAgent", BehaviorTreeTypes::Dog);
	dog->set_position(Vec3(40.0f, 0.0f, 40.0f));
	dog->set_color(Vec3(1.0f, 1.0f, 0.2f));
	dog->set_movement_speed(30.0f);

	auto owner = agents->create_behavior_agent("OwnerAgent", BehaviorTreeTypes::Owner);
	owner->set_position(Vec3(15.0f, 0.0f, 15.0f));
	owner->set_color(Vec3(1.0f, 0.2f, 1.0f));
	owner->set_movement_speed(20.0f);

	// you can technically load any map you want, even create your own map file,
	// but behavior agents won't actually avoid walls or anything special, unless you code that yourself
	// that's the realm of project 2 though
	terrain->goto_map(0);

	// you can also enable the pathing layer and set grid square colors as you see fit
	// works best with map 0, the completely blank map
	terrain->pathLayer.set_enabled(true);
	terrain->pathLayer.set_value(0, 0, Colors::Red);

	// camera position can be modified from this default as well
	auto camera = agents->get_camera_agent();
	camera->set_position(Vec3(-62.0f, 70.0f, terrain->mapSizeInWorld * 0.5f));
	camera->set_pitch(0.610865); // 35 degrees
}