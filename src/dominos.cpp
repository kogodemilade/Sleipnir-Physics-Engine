#include "Sleipnir/world.hpp"
#include "Sleipnir/body.hpp"
#include "Sleipnir/links.hpp"
#include "Sleipnir/collide_fine.hpp"
#include "Sleipnir/collide_coarse.hpp"
#include "Sleipnir/core.hpp"
#include "Sleipnir/precision.hpp"
// #include "Yggdrasil/ygg/engine.hpp"
#include "Sleipnir/ragdoll.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <iostream>
#include <thread>
#include <chrono>
#include <raylib.h>
#include <rlgl.h>
#include <vector>
#include <string.h>


static inline ::Vector3 ToRayLib(const cyclone::Vector3& v) {
return { v.x, v.y, v.z };
}

static inline ::Quaternion ToRayLib(const cyclone::Quaternion& q) {
    return { q.x, q.y, q.z, q.w };
}

void DrawRigidBox(
    const cyclone::RigidBody& body,
    const cyclone::Vector3& halfSize,
    Color color
) {
    // ::Vector3 pos = ToRayLib(body.getPosition());
    // cyclone::Quaternion q = body.getOrientation();
    
    cyclone::Matrix4 transform = body.getPosandOrient(); 
    
    // Convert to raylib Matrix (4x4, column-major)
    // raylib uses column-major, so we need to transpose the rotation part
    ::Matrix rayMatrix = {
        transform.data[0], transform.data[4], transform.data[8],  0.0f,  // First COLUMN: r00, r10, r20
        transform.data[1], transform.data[5], transform.data[9],  0.0f,  // Second COLUMN: r01, r11, r21
        transform.data[2], transform.data[6], transform.data[10], 0.0f,  // Third COLUMN: r02, r12, r22
        transform.data[3], transform.data[7], transform.data[11], 1.0f   // Fourth COLUMN: tx, ty, tz
    };
    // Begin 3D mode with custom transform
    rlPushMatrix();
    rlMultMatrixf((float*)&rayMatrix);
    
    // Draw the cube centered at origin
    DrawCube({0, 0, 0}, halfSize.x * 2, halfSize.y * 2, halfSize.z * 2, color);
    DrawCubeWires({0, 0, 0}, halfSize.x * 2, halfSize.y * 2, halfSize.z * 2, BLACK);
    
    rlPopMatrix();
}


//Function to create Dominos
void createDominos(unsigned numOfBoxes, std::vector<cyclone::RigidBody*> &Bodies, std::vector<cyclone::Box*> &Boxes, cyclone::World &world_, cyclone::real mass_, cyclone::real height_ratio, cyclone::real spacing, cyclone::real spacing_ratio, const cyclone::Vector3 &dimensions){
    for (int i =1; i<numOfBoxes+1; i++){

        cyclone::Vector3 rotational_shift_real(0, 0.1*(i-1), 0);
        

        cyclone::Quaternion angular_position = rotational_shift_real.toQuaternion();
        cyclone::real lateral_shift = spacing*(i*0.4);
        //Convenience
        cyclone::Matrix3 IdentityMatrix, ZeroMatrix3;
        IdentityMatrix.identityMatrix();
        ZeroMatrix3.setZero();

        cyclone::Matrix4 ZeroMatrix4;
        ZeroMatrix4.setZero();

        cyclone::real x_pos = dimensions.x*i + spacing_ratio*i*spacing;
        cyclone::real height = dimensions.y+height_ratio*i;
        // cyclone::Vector3 dominoPos = cyclone::Vector3(x_pos, dimensions.y*0.5, 1);
        // dominoPos.x+= dimensions.x*i + spacingRatio*i*spacing;
        cyclone::Quaternion dominoOrient = angular_position;
        cyclone::real mass = mass_ ;
        cyclone::RigidBody* domino = new cyclone::RigidBody;
        domino->setMass(mass);
        // domino->setPosition(dominoPos);
        domino->setOrientation(dominoOrient);
        // cyclone::RigidBody domino((cyclone::real)mass, dominoPos, dominoOrient, (cyclone::real)1.0);
        domino->setDimension(cyclone::Vector3(dimensions.z, dimensions.x, height));   //z,x,y ,, d,w,h

        cyclone::real dominoix = domino->getDimension('h')*domino->getDimension('h') + domino->getDimension('d')*domino->getDimension('d');
        cyclone::real dominoiy = domino->getDimension('w')*domino->getDimension('w') + domino->getDimension('d')*domino->getDimension('d');
        cyclone::real dominoiz = domino->getDimension('w')*domino->getDimension('w') + domino->getDimension('h')*domino->getDimension('h');

        domino->setSize(domino->getDimensions().magnitude());
        domino->setPosition(cyclone::Vector3(x_pos, height*0.5, lateral_shift*i));

        cyclone::Matrix3 dominoInertiaMatrix;
        dominoInertiaMatrix.setDiagonal(dominoix, dominoiy, dominoiz);
        dominoInertiaMatrix = dominoInertiaMatrix * (domino->getMass()/12);
        domino->setInertiaTensor(dominoInertiaMatrix);
        domino->name = (char)i;
        domino->setSleepEpsilon(0.02);
        domino->calculateDerivedData();

        cyclone::Box* dominoBox = new cyclone::Box;
        dominoBox->body = domino;    
        dominoBox->halfSize = cyclone::Vector3(domino->getDimension('w')*0.5, domino->getDimension('h')*0.5, domino->getDimension('d')*0.5);
        cyclone::Matrix4 offset;

        offset.setOrientAndPos(ZeroMatrix3, cyclone::Vector3(0,0,0));
        dominoBox->offset = offset;
        dominoBox->calculateInternals();
        dominoBox->bindPrimitive();
        domino->setSleepEpsilon(0.04);
        domino->setAwake(0);
        Bodies.push_back(domino);
        Boxes.push_back(dominoBox);
    }
}



using namespace cyclone;
int main(){

InitWindow(1280, 800, "Cyclone Physics Debug (raylib)");
SetTargetFPS(60);

Camera3D camera{};
camera.position = { 0.0f, 6.0f, 8.0f };
camera.target   = { 0.0f, 2.0f, 0.0f };
camera.up       = { 0.0f, 1.0f, 0.0f };
camera.fovy     = 60.0f;
camera.projection = CAMERA_PERSPECTIVE;

World world(1000, 1000);
cyclone::Gravity gravity(cyclone::Vector3(0, -9.81f, 0));

//Convenience
Matrix3 IdentityMatrix, ZeroMatrix3;
IdentityMatrix.identityMatrix();
ZeroMatrix3.setZero();

Matrix4 ZeroMatrix4;
ZeroMatrix4.setZero();


/*Start some rendering stuff*/

auto lastTime = std::chrono::high_resolution_clock::now();

/*End some rendering stuff*/

// Create ground
cyclone::RigidBody ground;
ground.setMass(0);
ground.setPosition(cyclone::Vector3(0,0,0));
ground.setSize(20.0f);
cyclone::Vector3 groundInertiaX(0.0, 0.0, 0.0);
cyclone::Vector3 groundInertiaY(0.0, 0.0, 0.0);
cyclone::Vector3 groundInertiaZ(0.0, 0.0, 0.0);
cyclone::Matrix3 groundTensor;
groundTensor.setComponents(groundInertiaX, groundInertiaY, groundInertiaZ);
ground.setInvInertiaTensor(groundTensor);
ground.name = "ground"; //Change to getter/setter
ground.setState(0);
ground.calculateDerivedData();

cyclone::Plane groundPlane;
groundPlane.body = &ground;
groundPlane.normal = cyclone::Vector3(0, 1, 0);
groundPlane.offset = (cyclone::real)0;
groundPlane.bindPrimitive();
groundPlane.calculateInternals();
world.addBodies(&ground, 1);


std::vector<cyclone::Box*> boxes;
std::vector<cyclone::RigidBody*> bodies;


createDominos(10, bodies, boxes, world, 10, 0.05, 0.05, 1.1, cyclone::Vector3(0.05, 0.5, 0.2));

//add gravity
for (auto body: bodies){
    world.addBodies(body);
    world.registry.add(body, &gravity);
}





int i = 0;
const float physics_dt = 0.008f;            
float accumulator = 0.0f;
auto previous = std::chrono::high_resolution_clock::now();

int iter = 0;
while (!WindowShouldClose()) {
    auto now = std::chrono::high_resolution_clock::now();
    float frameTime = std::chrono::duration<float>(now - previous).count();
    previous = now;

    // clamp to avoid spiral of death if frameTime huge
    if (frameTime > 0.25f) frameTime = 0.25f;

    accumulator += frameTime;

    // Step physics with fixed substeps,
    while (accumulator >= physics_dt) {
        world.startFrame();
        
        if(IsKeyPressed(KEY_K)){
        bodies[0]->addForceAtBodyPoint(cyclone::Vector3(-400, 0, 0), cyclone::Vector3(-bodies[0]->getDimension('w')*0.99, bodies[0]->getDimension('h')*0.99, 0));
        }
            // if( IsKeyDown(KEY_K)){
            world.runPhysics(physics_dt);
        // }

        accumulator -= physics_dt;
        iter++;
    }

    if ((iter % 10) == 0) {
    for (auto b: world.rigidBodies){
    std::cout << "iteration: " << iter*0.01 << "secs\n";
    std::cout<<"Body: "<<b->name<<std::endl;
    std::cout << "Position: x=" << b->getPosition().x
                << " y=" << b->getPosition().y << " z="<<b->getPosition().z << "\n";
    std::cout << "Velocity: x=" << b->getVelocity().x
                << " y=" << b->getVelocity().y << " z="<<b->getVelocity().z<<"\n";
    std::cout << "Orientation: x=" << b->getOrientation().toEulerAngles().x << " y="<<b->getOrientation().toEulerAngles().y << " z="<<b->getOrientation().toEulerAngles().z << "\n";
    std::cout << "Acceleration: x=" << b->getAcceleration().x
                << " y=" << b->getAcceleration().y << "\n\n";
    
    }}

    // Rendering section unchanged (use bodies' current transforms)

    UpdateCamera(&camera, CAMERA_FREE);

    BeginDrawing();
    ClearBackground({ 38, 38, 46, 255 });

    BeginMode3D(camera);

    DrawPlane({0,0,0}, {40,40}, WHITE);

    for (auto box: boxes){
    DrawRigidBox(
        *(box->body),
        box->halfSize,
        RED
    );}


    EndMode3D();

    DrawFPS(20, 20);
    EndDrawing();
}

for(auto b: bodies){
    delete b;
}
return 0;
}

/*Create Ground*/
/*Create each body*/
/*Create each joint*/