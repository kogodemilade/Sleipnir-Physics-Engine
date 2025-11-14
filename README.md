A physics engine is basically a massive calculator that calculates the position, velocities and accelerations of simulated objects. Physics engines are used in games, simulations and procedural animation.
In general, the 2 main parts of the engine are the force resolution and contact resolution. Force resolution deals with handling applied forces, while contact resolution deals with handling collisions and contacts
(How one object 'knows' it;s hit another, and the calculations to separate them or keep them from entering each other). How forces are resolved are relatively the same across different physics engines, but collision resolution varies. 
The method that's used here is using impulse, so it is an Impulse-based physics engine.

This is a hobby project I chose to practice my C++ skills. It's an adaptation from Ian Millington's Cyclone engine from his 'game physics engine development' book. 
There are quite a few liberties taken from the original that makes it comply with more modern C++, and have a more consistent programming style (in my opinion), amongst a few other decisions and implementation deviations.


Sleipnir is the 8-legged horse Odin rides from Norse mythology. I thought it was cool and ran with it.

There are quite a few things to work on and improve upon. Feel free to fork and submit pull requests.
