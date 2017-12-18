
#include "../../revisit-logger/cpp/logger.hpp"

#include <dart/dart.hpp>
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace Eigen;

#include <dart/gui/gui.hpp>

#include <iostream>
using std::cout;
using std::endl;


// Centimers conversion from SI units
constexpr long double operator"" _cm (long double meters) {
    return meters * 0.01;
}


// Notation for SI density
constexpr long double operator"" _kg_per_m3 (long double density) {
    return density;
}




class UGVWindow : public dart::gui::SimWindow
{
public:
    UGVWindow(WorldPtr world)
    {
        setWorld(world);
    }

    void keyboard(unsigned char key, int x, int y) override
    {
        switch(key)
        {
            default:
                SimWindow::keyboard(key, x, y);
        }
    }

    void timeStepping() override
    {
        // Step the simulation forward
        SimWindow::timeStepping();
    }
};



int main(int argc, char *argv[])
{
    // General parameters
    constexpr double material_density = 700.0_kg_per_m3;
    constexpr double vertical_offset = 50.0_cm;
    constexpr double restitution = 0.75;

    // Chassis parameters
    constexpr double wheel_base = 12.0_cm;
    constexpr double track_width = 100.0_cm;
    constexpr double chassis_height = 4.0_cm;
    const std::string chassis_name{"chassis"};
    const Vector3d chassis_dimensions(wheel_base, chassis_height, track_width);

    // Wheel parameters
    constexpr double wheel_radius = 22.5_cm;
    constexpr double wheel_thickness = 21.5_cm;
    const std::string wheel_name{"wheel"};


    //
    // Create the UGV skeleton
    //

    auto ugv = Skeleton::create("ugv");


    //
    // Create the chassis as part of the UGV and attach it to the world
    //

    ShapePtr chassis_shape(new BoxShape(chassis_dimensions));
    const double chassis_mass = material_density * chassis_shape->getVolume();

    // Setup the chassis properties
    FreeJoint::Properties chassis_joint_prop;
    chassis_joint_prop.mName = chassis_name + "_joint";

    BodyNode::Properties chassis_body_prop;
    chassis_body_prop.mName = chassis_name + "_body";
    chassis_body_prop.mRestitutionCoeff = restitution;
    chassis_body_prop.mInertia.setMass(chassis_mass);
    chassis_body_prop.mInertia.setMoment(chassis_shape->computeInertia(chassis_mass));

    // Create the joint-node pair
    FreeJoint* chassis_joint;
    BodyNode* chassis_body;
    std::tie(chassis_joint, chassis_body) = ugv->createJointAndBodyNodePair<FreeJoint>(
        nullptr, chassis_joint_prop, chassis_body_prop);

    // Set the shape of the body
    chassis_body->createShapeNodeWith<CollisionAspect, DynamicsAspect, VisualAspect>(chassis_shape);


    //
    // Create the wheels and attach them to the chassis
    //

    ShapePtr wheel_shape(new CylinderShape(wheel_radius, wheel_thickness));
    const double wheel_mass = material_density * wheel_shape->getVolume();

    // Setup the wheel properties
    RevoluteJoint::Properties wheel_joint_prop;
    wheel_joint_prop.mName = wheel_name + "_joint";

    BodyNode::Properties wheel_body_prop;
    wheel_body_prop.mName = wheel_name + "_body";
    wheel_body_prop.mRestitutionCoeff = restitution;
    wheel_body_prop.mInertia.setMass(wheel_mass);
    wheel_body_prop.mInertia.setMoment(wheel_shape->computeInertia(wheel_mass));

    // Create the joint-node pair
    RevoluteJoint* wheel_joint;
    BodyNode* wheel_body;
    std::tie(wheel_joint, wheel_body) =
        ugv->createJointAndBodyNodePair<RevoluteJoint>(
            chassis_body, wheel_joint_prop, wheel_body_prop);

    // Set the shape of the body
    wheel_body->createShapeNodeWith<CollisionAspect, DynamicsAspect, VisualAspect>(wheel_shape);


    // cout << wheel_joint->getAxis() << endl; --> 0 0 1

    //
    // Create a ground plane
    //

    auto ground_depth = 1.0;

    auto ground = Skeleton::create("ground");
    auto ground_body = ground->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
    ground_body->setRestitutionCoeff(restitution);
    ShapePtr ground_box(new BoxShape(Vector3d(100, ground_depth, 100)));
    ground_body->createShapeNodeWith<CollisionAspect, DynamicsAspect, VisualAspect>(ground_box);

    // Shift the ground so that its top is at y=0
    Isometry3d ground_tf(Isometry3d::Identity());
    ground_tf.translation() = Vector3d(0.0, -ground_depth / 2.0, 0.0);
    ground_body->getParentJoint()->setTransformFromParentBodyNode(ground_tf);


    //
    // Create the world and add the ugv skeleton
    //

    WorldPtr world(new World);

    world->addSkeleton(ugv);
    world->addSkeleton(ground);
    world->setGravity(Vector3d(0.0, -9.80665, 0.0));

    // Position the UGV above ground
    Vector6d ugv_positions(Vector6d::Zero());
    ugv_positions[4] = vertical_offset;
    chassis_joint->setPositions(ugv_positions);


    //
    // Use the DART gui interface to control the world
    //
    UGVWindow window(world);

    cout << "Some info about keyboard commands." << endl;

    // Initialize glut, initialize the window, and begin the glut event loop
    glutInit(&argc, argv);
    window.initWindow(640, 480, "UGV");
    glutMainLoop();


    // //
    // // Simulate the world for some amount of time
    // //

    // constexpr double TIME_STOP = 10;
    // constexpr double TIME_STEP = 0.001;
    // constexpr double VIS_STEP = 1.0 / 30.0;
    // constexpr double VIS_SCALE = 1000;
    // world->setTimeStep(TIME_STEP);

    // // Create the Revisit logger
    // revisit::logger rl(0.0, VIS_STEP, TIME_STOP);

    // rl.add_box(chassis_name,
    //     chassis_dimensions.x() * VIS_SCALE,
    //     chassis_dimensions.y() * VIS_SCALE,
    //     chassis_dimensions.z() * VIS_SCALE);

    // rl.add_cylinder(wheel_name,
    //     wheel_radius * VIS_SCALE,
    //     wheel_thickness * VIS_SCALE);

    // double next_vis_output_time = VIS_STEP;
    // while (world->getTime() < TIME_STOP + TIME_STEP/2.0) {
    //     world->step();

    //     if (world->getTime() > next_vis_output_time) {

    //         auto chassis_T = chassis_body->getTransform();
    //         auto ch_t = chassis_T.translation() * VIS_SCALE;
    //         Quaterniond ch_q(chassis_T.rotation());

    //         rl.add_frame(chassis_name,
    //             ch_t.x(), ch_t.y(), ch_t.z(),
    //             ch_q.x(), ch_q.y(), ch_q.z(), ch_q.w());

    //         auto wheel_T = wheel_body->getTransform();
    //         auto wh_t = wheel_T.translation() * VIS_SCALE;
    //         Quaterniond wh_q(wheel_T.rotation());

    //         rl.add_to_frame(wheel_name,
    //             wh_t.x(), wh_t.y(), wh_t.z(),
    //             wh_q.x(), wh_q.y(), wh_q.z(), wh_q.w());

    //         next_vis_output_time += VIS_STEP;
    //     }

    // }

    // // Passing false prints a compact JSON representation
    // cout << rl.to_string() << endl;
    // return EXIT_SUCCESS;
}
