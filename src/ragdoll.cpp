#include "Sleipnir/world.hpp"
#include "Sleipnir/body.hpp"
#include "Sleipnir/links.hpp"
#include "Sleipnir/collide_fine.hpp"
#include "Sleipnir/collide_coarse.hpp"
#include "Sleipnir/core.hpp"
#include "Sleipnir/precision.hpp"
#include "Yggdrasil/ygg/engine.hpp"
#include "Sleipnir/ragdoll.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <iostream>
#include <thread>
#include <chrono>


/*Start rendering code*/
Ygg::RenderEngine engine;
glm::mat4 projection;
Ygg::Camera cam = engine.createCamera({0.0f, 6.0f, 8.0f});

void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void mouse_callback(GLFWwindow *window, double xpos, double ypos);
void scroll_callback(GLFWwindow *window, double xOffset, double yOffset);
/*End rendering code*/


using namespace cyclone;
int main(){
World world(1000, 20);
cyclone::Gravity gravity(cyclone::Vector3(0, -9.81f, 0));

//Convenience
Matrix3 IdentityMatrix, ZeroMatrix3;
IdentityMatrix.identityMatrix();
ZeroMatrix3.setZero();

Matrix4 ZeroMatrix4;
ZeroMatrix4.setZero();


/*Start some rendering stuff*/
if (engine.initGL("../shaders/vShader.glsl", "../shaders/fShader2.glsl") != 0) {
    return -1;
}

GLFWwindow *window = engine.getWindow();

// simple GL state
// glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

//register scroll and mouse callback
glfwSetCursorPosCallback(window, mouse_callback);
glfwSetScrollCallback(window, scroll_callback);

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
ground.calculateDerivedData();

cyclone::Plane groundPlane;
groundPlane.body = &ground;
groundPlane.normal = cyclone::Vector3(0, 1, 0);
groundPlane.offset = (cyclone::real)0;
groundPlane.bindPrimitive();
groundPlane.calculateInternals();

world.addBodies(&ground, 1);

//Create each body. For now, only head, torso, legs and arms.
//Right Leg
Vector3 rLegPos = Vector3(0,1.5,0);
// Vector3 rLegPos = Vector3(0,10.5,0);
Quaternion rLegOrient = Quaternion(1,0,0,0);
RigidBody rLeg((real)10.0, rLegPos, rLegOrient, (real)3.1);
rLeg.setDimension(Vector3(1,1,3));   //z,x,y ,, d,w,h
real rLegix = rLeg.getDimension('h')*rLeg.getDimension('h') + rLeg.getDimension('d')*rLeg.getDimension('d');
real rLegiy = rLeg.getDimension('w')*rLeg.getDimension('w') + rLeg.getDimension('d')*rLeg.getDimension('d');
real rLegiz = rLeg.getDimension('w')*rLeg.getDimension('w') + rLeg.getDimension('h')*rLeg.getDimension('h');

Matrix3 rLegInertiaMatrix;
rLegInertiaMatrix.setDiagonal(rLegix, rLegiy, rLegiz);
rLegInertiaMatrix = rLegInertiaMatrix * (rLeg.getMass()/12);
rLeg.setInertiaTensor(rLegInertiaMatrix);
rLeg.name = "Right Leg";
rLeg.setSleepEpsilon(0.02);
rLeg.calculateDerivedData();

cyclone::Box rLegBox;
rLegBox.body = &rLeg;    
rLegBox.halfSize = Vector3(0.5, 1.5, 0.5);
Matrix4 offset;

offset.setOrientAndPos(ZeroMatrix3, Vector3(0,0,0));
rLegBox.offset = offset;
rLegBox.calculateInternals();
rLegBox.bindPrimitive();
rLeg.setAwake(1);
world.addBodies(&rLeg);
world.registry.add(&rLeg, &gravity);


// Left Leg
Vector3 lLegPos = Vector3(2,1.5,0);
Quaternion lLegOrient = Quaternion(1,0,0,0);
RigidBody lLeg((real)10.0, lLegPos, lLegOrient, (real)3.1);
lLeg.setDimension(Vector3(1,1,3));
real lLegix = lLeg.getDimension('h')*lLeg.getDimension('h') + lLeg.getDimension('d')*lLeg.getDimension('d');
real lLegiy = lLeg.getDimension('w')*lLeg.getDimension('w') + lLeg.getDimension('d')*lLeg.getDimension('d');
real lLegiz = lLeg.getDimension('w')*lLeg.getDimension('w') + lLeg.getDimension('h')*lLeg.getDimension('h');

Matrix3 lLegInertiaMatrix;
lLegInertiaMatrix.setDiagonal(lLegix, lLegiy, lLegiz);
lLegInertiaMatrix = lLegInertiaMatrix * (lLeg.getMass()/12);
lLeg.setInertiaTensor(lLegInertiaMatrix);
lLeg.name = "Left Leg";
lLeg.setSleepEpsilon(0.02);
lLeg.calculateDerivedData();

cyclone::Box lLegBox;
lLegBox.body = &lLeg;    
lLegBox.halfSize = Vector3(0.5, 1.5, 0.5);
lLegBox.bindPrimitive();
lLegBox.calculateInternals();
lLeg.setAwake(1);

world.addBodies(&lLeg);
world.registry.add(&lLeg, &gravity);


//Torso
Vector3 torsoPos = Vector3(1,4.51,0);
Quaternion torsoOrient = Quaternion(1,0,0,0);
RigidBody torso((real)7.0, torsoPos, torsoOrient, (real)3.5);
torso.setDimension(Vector3(1.5,3,3)); //z,x,y
real torsoix = torso.getDimension('h')*torso.getDimension('h') + torso.getDimension('d')*torso.getDimension('d');
real torsoiy = torso.getDimension('w')*torso.getDimension('w') + torso.getDimension('d')*torso.getDimension('d');
real torsoiz = torso.getDimension('w')*torso.getDimension('w') + torso.getDimension('h')*torso.getDimension('h');

Matrix3 torsoInertiaMatrix;
torsoInertiaMatrix.setDiagonal(torsoix, torsoiy, torsoiz);
torsoInertiaMatrix = torsoInertiaMatrix * (torso.getMass()/12);
torso.setInertiaTensor(torsoInertiaMatrix);
torso.name = "Torso";
torso.calculateDerivedData();

cyclone::Box torsoBox;
torsoBox.body = &torso;    
torsoBox.halfSize = Vector3(1.5, 1.5, 0.75);

torsoBox.offset = offset;
torsoBox.bindPrimitive();
torsoBox.calculateInternals();
torso.setAwake(1);

world.addBodies(&torso);
world.registry.add(&torso, &gravity);


// //Head
// Vector3 headPos = Vector3(1,6.51,0);
// Quaternion headOrient = Quaternion(1,0,0,0);
// RigidBody head((real)3.0, headPos, headOrient, (real)1.2);
// head.setDimension(Vector3(1,1,1));
// real headix = head.getDimension('h')*head.getDimension('h') + head.getDimension('d')*head.getDimension('d');
// real headiy = head.getDimension('w')*head.getDimension('w') + head.getDimension('d')*head.getDimension('d');
// real headiz = head.getDimension('w')*head.getDimension('w') + head.getDimension('h')*head.getDimension('h');

// Matrix3 headInertiaMatrix;
// headInertiaMatrix.setDiagonal(headix, headiy, headiz);
// headInertiaMatrix = headInertiaMatrix * (head.getMass()/12);
// head.setInertiaTensor(headInertiaMatrix);
// head.name = "Head";
// head.calculateDerivedData();

// cyclone::Box headBox;
// headBox.body = &head;    
// headBox.halfSize = Vector3(0.5, 0.5, 0.5);
// headBox.bindPrimitive();
// headBox.calculateInternals();

// world.addBodies(&head);
// world.registry.add(&head, &gravity);


// //rArm
// Vector3 rArmPos = Vector3(-1.51 , 5.0 ,0);
// Quaternion rArmOrient = Quaternion(1,0,0,0);
// RigidBody rArm((real)4.0, rArmPos, rArmOrient, (real)2.2);
// rArm.setDimension(Vector3(1,2,1)); //zxy
// real rArmix = rArm.getDimension('h')*rArm.getDimension('h') + rArm.getDimension('d')*rArm.getDimension('d');
// real rArmiy = rArm.getDimension('w')*rArm.getDimension('w') + rArm.getDimension('d')*rArm.getDimension('d');
// real rArmiz = rArm.getDimension('w')*rArm.getDimension('w') + rArm.getDimension('h')*rArm.getDimension('h');

// Matrix3 rArmInertiaMatrix;
// rArmInertiaMatrix.setDiagonal(rArmix, rArmiy, rArmiz);
// rArmInertiaMatrix = rArmInertiaMatrix * (rArm.getMass()/12);
// rArm.setInertiaTensor(rArmInertiaMatrix);
// rArm.name = "Right arm";
// rArm.calculateDerivedData();

// cyclone::Box rArmBox;
// rArmBox.body = &rArm;    
// rArmBox.halfSize = Vector3(1, 0.5, 0.5);
// rArmBox.bindPrimitive();
// rArmBox.calculateInternals();

// world.addBodies(&rArm);
// world.registry.add(&rArm, &gravity);


// //lArm
// Vector3 lArmPos = Vector3(3.51,5.0,0);
// Quaternion lArmOrient = Quaternion(1,0,0,0);
// RigidBody lArm((real)4.0, lArmPos, lArmOrient, (real)2.2);
// lArm.setDimension(Vector3(1,2,1));

// real lArmix = lArm.getDimension('h')*lArm.getDimension('h') + lArm.getDimension('d')*lArm.getDimension('d');
// real lArmiy = lArm.getDimension('w')*lArm.getDimension('w') + lArm.getDimension('d')*lArm.getDimension('d');
// real lArmiz = lArm.getDimension('w')*lArm.getDimension('w') + lArm.getDimension('h')*lArm.getDimension('h');

// Matrix3 lArmInertiaMatrix;
// lArmInertiaMatrix.setDiagonal(lArmix, lArmiy, lArmiz);
// lArmInertiaMatrix = lArmInertiaMatrix * (lArm.getMass()/12);
// lArm.setInertiaTensor(lArmInertiaMatrix);
// lArm.name = "Left arm";
// lArm.calculateDerivedData();

// cyclone::Box lArmBox;
// lArmBox.body = &lArm;    
// lArmBox.halfSize = Vector3(1, 0.5, 0.5);
// lArmBox.bindPrimitive();
// lArmBox.calculateInternals();

// world.addBodies(&lArm);
// world.registry.add(&lArm, &gravity);



//Create Joints
// PositionJoint lLegtoTorso(&lLeg, &torso, Vector3( 0, 1.5 ,0), Vector3(1, -1.5 , 0), 0.002);

// PositionJoint rLegtoTorso(&rLeg, &torso, Vector3( 0, 1.5 ,0), Vector3(-1, -1.5 , 0), 0.002);

// PositionJoint lArmToTorso(&lArm, &torso, Vector3(-1, 0, 0), Vector3(1.5, 1, 0), 0.002);

// PositionJoint rArmToTorso(&rArm, &torso, Vector3(1, 0, 0), Vector3(-1.5, 1, 0), 0.002);

// PositionJoint headtoTorso(&head, &torso, Vector3(0, -0.5, 0), Vector3(0, 1.5, 0), 0.002);

// world.addContactGenerator(&lLegtoTorso);
// world.addContactGenerator(&rLegtoTorso);
// world.addContactGenerator(&lArmToTorso);
// world.addContactGenerator(&rArmToTorso);
// world.addContactGenerator(&headtoTorso);




/*Start rendering code*/
// // create a few demo meshes
Ygg::Mesh floor = engine.createBox(ground.getPosition().toGlm(), ground.getOrientation().toGlm(), ground.getSize(), 0.0001f, ground.getSize(), {0.7f, 1.7f, 0.0f}); //ground

Ygg::Mesh rLegMesh = engine.createBox(rLeg.getPosition().toGlm(), rLeg.getOrientation().toGlm(), 2.0f*rLeg.getDimension('w'), 2.0f*rLeg.getDimension('h'), 2.0f*rLeg.getDimension('d'), {1.0f, 0.1f, 0.1f}); //red

Ygg::Mesh lLegMesh = engine.createBox(lLeg.getPosition().toGlm(), lLeg.getOrientation().toGlm(), 2.0f*lLeg.getDimension('w'), 2.0f*lLeg.getDimension('h'), 2.0f*lLeg.getDimension('d'), {1.0f, 0.1f, 0.1f}); // red

// Ygg::Mesh rArmMesh = engine.createBox(rArm.getPosition().toGlm(), rArm.getOrientation().toGlm(), 2.0f*rArm.getDimension('w'), 2.0f*rArm.getDimension('h'), 2.0f*rArm.getDimension('d'), {0.1f, 1.0f, 0.1f});//green

// Ygg::Mesh lArmMesh = engine.createBox(lArm.getPosition().toGlm(), lArm.getOrientation().toGlm(), 2.0f*lArm.getDimension('w'), 2.0f*lArm.getDimension('h'), 2.0f*lArm.getDimension('d'), {0.1f, 1.0f, 0.1f});//green

Ygg::Mesh torsoMesh = engine.createBox(torso.getPosition().toGlm(), torso.getOrientation().toGlm(), 2.0f*torso.getDimension('w'), 2.0f*torso.getDimension('h'), 2.0f*torso.getDimension('d'), {0.1f, 0.1f, 1.0f}); // blue

// Ygg::Mesh headMesh = engine.createBox(head.getPosition().toGlm(), head.getOrientation().toGlm(), 2.0f*head.getDimension('w'), 2.0f*head.getDimension('h'), 2.0f*head.getDimension('d'), {1.0f, .0f, 1.0f}); //White

static Ygg::Line thread = engine.createLine();

/*End rendering code*/



int i = 0;
while(!glfwWindowShouldClose(window)){
    auto now = std::chrono::high_resolution_clock::now();
    float dt = std::chrono::duration<float>(now - lastTime).count();
    lastTime = now;

    world.startFrame();
    world.runPhysics(0.01);
    // if( (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)){
            //  world.runPhysics(0.01);
    for (auto b: world.rigidBodies){
    std::cout << "iteration: " << i*0.01 << "secs\n";
    std::cout<<"Body: "<<b->name<<std::endl;
    std::cout << "Position: x=" << b->getPosition().x
                << " y=" << b->getPosition().y << " z="<<b->getPosition().z << "\n";
    std::cout << "Velocity: x=" << b->getVelocity().x
                << " y=" << b->getVelocity().y << " z="<<b->getVelocity().z<<"\n";
    std::cout << "Orientation: x=" << b->getOrientation().toEulerAngles().x << " y="<<b->getOrientation().toEulerAngles().y << " z="<<b->getOrientation().toEulerAngles().z << "\n";
    std::cout << "Acceleration: x=" << b->getAcceleration().x
                << " y=" << b->getAcceleration().y << "\n\n";
    
    // for (auto data: anchor.getInvInertiaTensorWorld().data){
    // std::cout << "Inverse Inertia tensor world "<<data<<"\n";}
    // std::cout << "\n\n\n\n";
        std::cout<<"\n\n\n";
    }
// }


    /*Start rendering code*/
    cam.processInput(window, dt);

    // // render
    glClearColor(0.15f, 0.15f, 0.18f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // draw meshes (these meshes were baked with model transforms in createBox/createSphere)
    glm::mat4 view = cam.getViewMatrix();
    engine.drawMesh(floor, view, projection, cam.getCameraPos(), floor.model);
    engine.drawMesh(rLegMesh, view, projection, cam.getCameraPos(), rLeg.getPosandOrient().toGlm());
    engine.drawMesh(lLegMesh, view, projection, cam.getCameraPos(), lLeg.getPosandOrient().toGlm());
    // engine.drawMesh(rArmMesh, view, projection, cam.getCameraPos(), rArm.getPosandOrient().toGlm());
    // engine.drawMesh(lArmMesh, view, projection, cam.getCameraPos(), lArm.getPosandOrient().toGlm());
    engine.drawMesh(torsoMesh, view, projection, cam.getCameraPos(), torso.getPosandOrient().toGlm());
    // engine.drawMesh(headMesh, view, projection, cam.getCameraPos(), head.getPosandOrient().toGlm());


    // glm::vec3 p1 = anchor.getPosition().toGlm();
    // glm::vec3 p2 = b.getPosition().toGlm();
    // glm::vec3 ropecolor = {0,0,0};
    // engine.updateLine(thread, p1, p2, ropecolor);
    // engine.drawLine(thread, view, projection, cam.getCameraPos());

    glfwSwapBuffers(window);
    glfwPollEvents();
    /*Stop rendering code*/



    if (i == (int)0.66*100){
        unsigned _dosmth;
        _dosmth += 1; //Dummy code for some debugging
    }


    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    i++;

}

return 0;
}


/*Some other rendering stuff*/
void framebuffer_size_callback(GLFWwindow *window, int width, int height){
    glViewport(0, 0, width, height);
    projection = glm::perspective(glm::radians(cam.getFov()), (float)width/height, 0.1f, 100.0f);

}

void mouse_callback(GLFWwindow *window, double xpos, double ypos){
    cam.processMouseMovement(window, xpos, ypos);
}

void scroll_callback(GLFWwindow *window, double xOffset, double yOffset){
    cam.processMouseScroll(static_cast<float>(yOffset), projection);
}
/*End rendering stuff*/

/*Create Ground*/
/*Create each body*/
/*Create each joint*/