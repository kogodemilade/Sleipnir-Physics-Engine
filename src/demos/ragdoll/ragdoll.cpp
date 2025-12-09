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

using namespace cyclone;
int main(
World world(1000, 1000);

Vector3 ragRoot(0.0f, 6.0f, 0.0f);
auto rag = cyclone::RagdollBuilder::createSimpleRagdoll(world, ragRoot);

while(1){
    
}

)