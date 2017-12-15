
#include <dart/dart.hpp>
#include "../revisit-logger/cpp/logger.hpp"
#include <iostream>

int main()
{
    constexpr double density = 1.0;
    constexpr double radius = 0.3;
    constexpr double restitution = 0.9;
    constexpr double damping = 0.0;
    constexpr double starting_height = 10.0;
    const std::string name{"sphere1"};

    //
    // Create the sphere skeleton (returns a shared_ptr)
    //

    dart::dynamics::SkeletonPtr sphere = dart::dynamics::Skeleton::create(name);


    //
    // Create a shape object to attach to the BodyNode (shared_ptr)
    //

    dart::dynamics::ShapePtr sphere_shape(new dart::dynamics::SphereShape(radius));


    //
    // Create joint properties for a free joint that connects the
    // sphere to the world. This is a free joint since the body does
    // not need any constraints with respect to the world.
    //

    dart::dynamics::FreeJoint::Properties sphere_joint_prop;

    // Joint damping coefficient (wind resistance in this case)
    // (Free joints have 6 DOF)
    sphere_joint_prop.mDampingCoefficients = Eigen::Vector6d::Constant(damping);


    //
    // Create body properties for the sphere
    //

    dart::dynamics::BodyNode::Properties sphere_body_prop;

    // Inertia
    const double sphere_mass = density * sphere_shape->getVolume();
    sphere_body_prop.mInertia.setMass(sphere_mass);
    sphere_body_prop.mInertia.setMoment(sphere_shape->computeInertia(sphere_mass));

    // Coefficient of restitution
    sphere_body_prop.mRestitutionCoeff = restitution;


    //
    // Add the sphere body and connect to the world
    //  - template parameters: JointType (required) and NodeType
    //  - parent body (null since there is no parent)
    //  - joint properties
    //  - body properties
    //

    dart::dynamics::FreeJoint* sphere_joint;
    dart::dynamics::BodyNode* sphere_body;

    std::tie(sphere_joint, sphere_body) =
        sphere->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
            nullptr, sphere_joint_prop, sphere_body_prop);


    //
    // Configure the BodyNode shape so that it has a collision aspect and
    // a dynamic aspect.
    //

    sphere_body->createShapeNodeWith<
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(
            sphere_shape);


    //
    // Set the initial position of the sphere.
    // The first three components are the orgientation using angle-axis
    // (logmap), the second three components are the translation.
    //

    Eigen::Vector6d positions(Eigen::Vector6d::Zero());
    positions[5] = starting_height;
    sphere_joint->setPositions(positions);


    //
    // Create the world object (shared_ptr) and add the sphere skeleton
    //

    dart::simulation::WorldPtr world(new dart::simulation::World);
    world->addSkeleton(sphere);


    //
    // Add a ground plane
    //

    auto ground = dart::dynamics::Skeleton::create("ground");

    auto ground_body = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(
        nullptr).second;

    ground_body->setRestitutionCoeff(restitution);

    dart::dynamics::ShapePtr ground_box(new dart::dynamics::BoxShape(
        Eigen::Vector3d(100, 100, 1)));

    ground_body->createShapeNodeWith<
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(
            ground_box);

    world->addSkeleton(ground);


    //
    // Simulate the world for some amount of time
    //

    constexpr double TIME_STOP = 10;
    constexpr double TIME_STEP = 0.001;
    constexpr double VIS_STEP = 1.0 / 100.0;
    constexpr double VIS_SCALE = 50;
    world->setTimeStep(TIME_STEP);

    // The logger uses a vertical y-axis (and cannot be changed, for now)
    revisit::logger rl(0.0, VIS_STEP, TIME_STOP);
    rl.add_group(name, "sphere", radius * 2 * VIS_SCALE);
    rl.add_frame(name, 0, starting_height * VIS_SCALE, 0, 0, 0, 0, 1);

    double next_vis_output_time = VIS_STEP;
    while (world->getTime() < TIME_STOP + TIME_STEP/2.0) {
        world->step();

        if (world->getTime() > next_vis_output_time) {

            // Ignore rotations and lateral movements
            rl.add_frame(name,
                0.0, sphere->getPosition(5) * VIS_SCALE, 0.0,
                0.0, 0.0, 0.0, 1.0);

            next_vis_output_time += VIS_STEP;
        }

    }

    // Passing false prints a compact JSON representation
    std::cout << rl.to_string(false) << std::endl;

    return EXIT_SUCCESS;
}
