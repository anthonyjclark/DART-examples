
#include "../revisit-logger/cpp/logger.hpp"

#include <dart/dart.hpp>
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace Eigen;

#include <dart/collision/bullet/bullet.hpp>

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

using std::string;
using std::vector;


// ----------------------------------------------------------------------------
// Properties used to create an object
struct ObjectProperties
{
    Vector3d dimensions;    // XYZ dimentions
    Vector3d offset;        // XYZ offset position
    double restitution;     // Material bounciness
    double density;         // Material density
    BodyNode* parent;       // Parent body node
    string name;
};


// ----------------------------------------------------------------------------
// Centimers conversion from SI units
constexpr long double operator"" _cm (long double meters) {
    return meters * 0.01;
}
constexpr long double operator"" _cm (unsigned long long meters) {
    return meters * 0.01;
}


// ----------------------------------------------------------------------------
// Notation for SI density
constexpr long double operator"" _kg_per_m3 (long double density) {
    return density;
}
constexpr long double operator"" _kg_per_m3 (unsigned long long density) {
    return density;
}


// ----------------------------------------------------------------------------
// Create the UGV's chassis
void add_chassis(SkeletonPtr skel, ObjectProperties & cp)
{
    ShapePtr shape{new BoxShape(cp.dimensions)};
    const double mass = cp.density * shape->getVolume();

    // Setup the joint properties
    FreeJoint::Properties joint_prop;
    joint_prop.mName = cp.name + "_joint";

    // Setup the body properties
    BodyNode::Properties body_prop;
    body_prop.mName = cp.name;
    body_prop.mRestitutionCoeff = cp.restitution;
    body_prop.mInertia.setMass(mass);
    body_prop.mInertia.setMoment(shape->computeInertia(mass));

    // Create the joint-node pair
    auto body_joint_pair = skel->createJointAndBodyNodePair<FreeJoint>(
        cp.parent, joint_prop, body_prop);

    // Set the shape of the body
    body_joint_pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);
}


// ----------------------------------------------------------------------------
// Create a UGV wheel
void add_wheel(SkeletonPtr skel, const ObjectProperties & wp)
{
    ShapePtr shape{new EllipsoidShape(wp.dimensions)};
    const double mass = wp.density * shape->getVolume();

    // Setup the wheel properties
    RevoluteJoint::Properties joint_prop;
    joint_prop.mName = wp.name + "_joint";

    Isometry3d tf(Isometry3d::Identity());
    tf.translation() = wp.offset;
    joint_prop.mT_ParentBodyToJoint = tf;

    // Setup the body properties
    BodyNode::Properties body_prop;
    body_prop.mName = wp.name;
    body_prop.mRestitutionCoeff = wp.restitution;
    body_prop.mInertia.setMass(mass);
    body_prop.mInertia.setMoment(shape->computeInertia(mass));

    // Create the joint-node pair
    auto body_joint_pair = skel->createJointAndBodyNodePair<RevoluteJoint>(
        wp.parent, joint_prop, body_prop);

    // Set the shape of the body
    body_joint_pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);

    // Set the actuator type of the wheel joint
    body_joint_pair.first->setActuatorType(Joint::VELOCITY);
}


// ----------------------------------------------------------------------------
// Update the logger with current body positions
void add_frame_to_rl(revisit::logger & logger, WorldPtr & world, double scale) {

    logger.new_frame();

    for (size_t skel_idx = 0; skel_idx < world->getNumSkeletons(); ++skel_idx) {

        auto skel = world->getSkeleton(skel_idx);
        for (const auto & bnode : skel->getBodyNodes()) {

            auto T = bnode->getTransform();
            auto trans = T.translation() * scale;
            Quaterniond quat(T.rotation());

            logger.add_to_frame(
                bnode->getName(),
                trans.x(), trans.y(), trans.z(),
                quat.x(), quat.y(), quat.z(), quat.w());
        }
    }
}


// ----------------------------------------------------------------------------
// Create the UGV's chassis
int main()
{
    // General parameters
    constexpr double material_density = 700_kg_per_m3;
    constexpr double material_restitution = 0.75;
    constexpr double vertical_offset = 20_cm;

    // Chassis parameters
    constexpr double wheel_base = 10_cm;
    constexpr double track_width = 12_cm;
    constexpr double chassis_height = 4_cm;
    const string chassis_name{"chassis"};
    const Vector3d chassis_dimensions{wheel_base, chassis_height, track_width};

    // Wheel parameters (modeled as an ellipsoid)
    constexpr double wheel_radius = 2.5_cm;
    constexpr double wheel_thickness = 1.5_cm;
    const Vector3d wheel_dimensions{wheel_radius * 2, wheel_radius * 2, wheel_thickness};


    //
    // Create the UGV skeleton
    //

    auto ugv = Skeleton::create("ugv");


    //
    // Create the chassis as part of the UGV and attach it to the world
    //

    ObjectProperties chassis_props{
        chassis_dimensions,
        Vector3d(0, 0, 0),
        material_restitution,
        material_density,
        nullptr,
        chassis_name
    };
    add_chassis(ugv, chassis_props);


    //
    // Create the wheels and attach them to the chassis
    //

    // A general wheel properties object (only the name and offset will need to be changed)
    ObjectProperties wp{
        wheel_dimensions,
        Vector3d(-1, 0, -1),
        material_restitution,
        material_density,
        ugv->getBodyNode(chassis_name),
        ""
    };

    vector<string> wheel_names{
        "front-right-wheel", "front-left-wheel", "back-right-wheel", "back-left-wheel"
    };

    // Save the wheel indicies so that they can be controlled
    vector<long> wheel_idxs;

    for (const auto & name : wheel_names) {

        wp.name = name;
        wp.offset.x() = wheel_base  / 2.0 * (name.find("back") == string::npos ? 1 : -1);
        wp.offset.z() = track_width / 2.0 * (name.find("left") == string::npos ? 1 : -1);

        add_wheel(ugv, wp);
        wheel_idxs.push_back(ugv->getDof(name + "_joint")->getIndexInSkeleton());
    }


    //
    // Create a ground plane
    //

    auto ground_depth = 1.0;

    auto ground = Skeleton::create("ground");
    auto ground_body = ground->createJointAndBodyNodePair<WeldJoint>().second;
    ground_body->setRestitutionCoeff(material_restitution);
    ground_body->setName("ground");
    ShapePtr ground_box(new BoxShape(Vector3d(100, ground_depth, 100)));
    ground_body->createShapeNodeWith<CollisionAspect, DynamicsAspect>(ground_box);

    // Shift the ground so that its top is at y=0
    Isometry3d ground_tf(Isometry3d::Identity());
    ground_tf.translate(Vector3d(0.0, -ground_depth / 2.0, 0.0));
    ground_body->getParentJoint()->setTransformFromParentBodyNode(ground_tf);


    //
    // Create the world and add the ugv skeleton
    //

    WorldPtr world(new World);

    // The Bullet collision detector uses primitives instead of meshes, which makes
    // it faster and more useful for this simple application.
    if (dart::collision::CollisionDetector::getFactory()->canCreate("bullet")) {
        world->getConstraintSolver()->setCollisionDetector(
            dart::collision::CollisionDetector::getFactory()->create("bullet"));
    } else {
        cerr << "NO BULLET" << endl;
    }

    world->addSkeleton(ugv);
    world->addSkeleton(ground);
    world->setGravity(Vector3d(0.0, -9.80665, 0.0));

    // Position the UGV above ground
    Vector6d ugv_positions(Vector6d::Zero());
    ugv_positions[4] = vertical_offset;
    ugv->getJoint(chassis_name + "_joint")->setPositions(ugv_positions);


    //
    // Simulate the world for some amount of time
    //

    constexpr double TIME_STOP = 10;
    constexpr double TIME_STEP = 0.005;
    constexpr double VIS_STEP = 1.0 / 30.0;
    constexpr double VIS_SCALE = 500;
    world->setTimeStep(TIME_STEP);

    // Create the Revisit logger
    revisit::logger rl(0.0, VIS_STEP, TIME_STOP);

    rl.add_box(chassis_name,
        chassis_dimensions.x() * VIS_SCALE,
        chassis_dimensions.y() * VIS_SCALE,
        chassis_dimensions.z() * VIS_SCALE);

    for (const auto & name : wheel_names) {
        rl.add_ellipsoid(name,
            wheel_dimensions.x() * VIS_SCALE,
            wheel_dimensions.y() * VIS_SCALE,
            wheel_dimensions.z() * VIS_SCALE);
    }


    //
    // Controllers
    //

    double next_vis_output_time = 0;
    while (world->getTime() < TIME_STOP + TIME_STEP/2.0) {

        world->step();

        auto wheel_speed = -10;
        ugv->setCommand(wheel_idxs.at(0), wheel_speed);
        ugv->setCommand(wheel_idxs.at(1), wheel_speed);
        ugv->setCommand(wheel_idxs.at(2), wheel_speed);
        ugv->setCommand(wheel_idxs.at(3), wheel_speed);


        if (world->getTime() > next_vis_output_time) {

            add_frame_to_rl(rl, world, VIS_SCALE);

            next_vis_output_time += VIS_STEP;
        }

    }

    // Passing false prints a compact JSON representation
    cout << rl.to_string() << endl;
    return EXIT_SUCCESS;
}
