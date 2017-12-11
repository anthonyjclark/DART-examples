
# Starting with DART (Dynamic Animation and Robotics Toolkit)

In this post, I will be showing how to simulating a falling sphere using [DART](https://dartsim.github.io/). For this simple example, I will not be worrying about any form of visualization. This post is mostly intended to help me document and remember how to setup a simple simulation.

I have used a variety of different simulation packages for my research. I've never been overly satisfied with the two that I have used most often: [Open Dynamics Engine (ODE)](https://www.ode-wiki.org/wiki/) and [MATLAB/Simulink](https://www.mathworks.com/). ODE *works*, but it is a game engine and its emphasis is on speed and less so on accuracy. Simulink, on the other hand, requires you to pay for it and MATLAB and it focuses on numerical simulation (not physical simulation). I'll likely still use the tools going forward, however [Jean-Baptiste Mouret's](https://members.loria.fr/JBMouret/) talk at [SimER](http://cis.gvsu.edu/~moorejar/SimER/) this past July convinced me to give DART a try. 

## Installing DART

On a mac, installing should be as simple as:

```bash
brew install dartsim6
```

However, I generally prefer to only install optional dependencies that I know I'll use. So, I first tried to install like so:

```bash
brew install dartsim6 --without-gui \
                      --without-gui-osg \
                      --without-optimizer-ipopt \
                      --without-optimizer-nlopt \
                      --without-planning \
                      --without-utils \
                      --without-utils-urdf
```

You can see all of the options with:

```bash
brew info dartsim6
```

Installing with [brew](https://brew.sh/) is the ideal scenario. However, on my system [I ran into a strange issue](https://github.com/dartsim/dart/issues/946).

So, I ended up installing like so:

```bash
hub clone dartsim/dart
cd dart/
git checkout tags/v6.3.0
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX:PATH=/path/to/local/installation ..
make install
```

I installed DART into a non-standard local location so that I could easily delete it later **after** successfully installing using brew (once the issue has been fixed). Installing to a non-standard location does require a bit of extra working during compiling. Specifically, I had to add the lib folder to `DYLD_LIBRARY_PATH`.


## A Falling Sphere

For this simple example, I am just going to have a sphere fall and then bounce on a static ground plane. The [source can be found in this repository](#), but I will be stepping through each part of the simple example here.

To create a sphere, we need to start with a `Skeleton`. 

```cpp
dart::dynamics::SkeletonPtr sphere = dart::dynamics::Skeleton::create(name);
```

A `Skeleton` acts as a container for several objects. Any shapes that you wants to be grouped together (for example, the chassis and wheels of a vehicle) can be added to a single skeleton.

Next, we need to create a `SphereShape`:

```cpp
dart::dynamics::ShapePtr sphere_shape(new dart::dynamics::SphereShape(radius));
```

This object represents a geometry that can be used to calculate sphere properties (volume, inertia tensor, etc.) and will eventually be used to create a `BodyNode` with dynamic and collisions aspects.

To create the actual sphere, we need to define a `Joint` and a `Body`. The `Joint` attaches the sphere to another `Body`, or to a `World` when the body doesn't need to connect to another `Body`, and one joint is required for every body. In this case, we want the sphere to free fall, so we attach it to the `World` using a `FreeJoint`.

To help initialize these entities, we can use **properties**:

```cpp
dart::dynamics::FreeJoint::Properties sphere_joint_prop;
sphere_joint_prop.mDampingCoefficients = Eigen::Vector6d::Constant(damping);

dart::dynamics::BodyNode::Properties sphere_body_prop;
sphere_body_prop.mInertia.setMass(sphere_mass);
sphere_body_prop.mInertia.setMoment(sphere_shape->computeInertia(sphere_mass));
sphere_body_prop.mRestitutionCoeff = restitution;
```

In the code above I have set a few of the properties. For a the sphere's `FreeJoint`, you can think of damping as adding wind resistance. In this case I have added damping to all 6 degrees-of-freedom (rotation and translation), which doesn't necessarily make complete physical sense, but it serves as a good example. Likewise, for the body I have added inertial properties and a coefficient of restitution.

Next, we create the `Joint` and `Body`:

```cpp
dart::dynamics::FreeJoint* sphere_joint;
dart::dynamics::BodyNode* sphere_body;

std::tie(sphere_joint, sphere_body) =
    sphere->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
        nullptr, sphere_joint_prop, sphere_body_prop);
```

The `createJointAndBodyNodePair` method adds a `Body` and `Joint` to a `Skeleton`. I am using `std::tie` since the method returns a `std::pair` and I don't like accessing members through `first` and `second`.

Now that we have a `Body` and a `Joint` we can set the sphere's properties.

```cpp
sphere_body->createShapeNodeWith<
    dart::dynamics::CollisionAspect,
    dart::dynamics::DynamicsAspect>(
        sphere_shape);

Eigen::Vector6d positions(Eigen::Vector6d::Zero());
positions[5] = starting_height;
sphere_joint->setPositions(positions);
```

The first method call sets the `Shape` of the sphere to the shape that we created above. The second method calls set the starting position of the sphere to `starting_height` above a ground plane (see below).

Finally, we need to create a world and add the sphere to it:

```cpp
dart::simulation::WorldPtr world(new dart::simulation::World);
world->addSkeleton(sphere);
```

At this point, we could add a few calls to `world->step()` and everything should work as expected. However, since the sphere falling for all-of-time is not very interesting we are going to add a ground plane.

## A Ground Plane

Since most of this code is repetitive, I am not going to go through this step-by-step:

```cpp
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
```

The main differences between the sphere and this ground plane is that the ground plane uses a `WeldJoint` to attach it to the world (since it does not move), and it uses a `BoxShape` instead of a `SphereShape`. *Note: DART does have a `PlaneShape`, but it is not supported by the default collision engine ([FCL](https://github.com/flexible-collision-library/fcl))--or at least DART doesn't support FCL's plane type yet.*

## Simulating the Sphere

Now that we have a sphere and a ground plane, all that is left is to run the simulation:

```cpp
constexpr double TIME_STOP = 10;
constexpr double TIME_STEP = 0.001;
world->setTimeStep(TIME_STEP);

while (world->getTime() < TIME_STOP) {
    world->step();
}
```

The figure below shows results from this simulation with different values for the coefficient of restitution and wind drag (`FreeJoint` damping).

![chart](chart.png)

## Notes

DART is still under active development and I wouldn't quite consider it completely mature (the API is still subject to rapid changes). Even in this short example I found a few oddities. First, DART has a default constructor for `BodyNode` properties (that can be passed to `createJointAndBodyNodePair`) called `BodyNode::AspectProperties()` but the corresponding `FreeJoint::AspectProperties()` cannot be passed to the same function. I'm sure this is my user error, but the naming convention leaves some to be desired.

I am also not a fan of how freely they mix raw pointers and smart pointers. `createJointAndBodyNodePair` returns raw pointers while most other methods in this example return `std::shared_ptr`.

## References

- [DART Documentation](https://dartsim.github.io/api/v6.1.1/index.html)
