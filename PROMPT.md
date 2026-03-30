The project description is as follows:
Implement a 2D physics simulator. The simulator should simulate circular balls and fixed, immovable walls. The balls should be influenced by gravity and should collide with the walls and with other balls in non-elastic collisions with restitution. If you don't know what that means, that's OK, Amp probably does.

Use SDL3 via C++. Running your application should set up an initial "scene" with about a thousand balls in some kind of container made up of wall pieces. Set up the initial scene so the balls bounce around for a while and then settle down. Ideally it'll also be fast enough that it's pleasant to look at.

The hard part is going to be making sure the balls don't end up overlapping or squeezing through the walls. You'll also sometimes see balls start vibrating really fast. Sometimes they vibrate fast and faster and eventually shoot off to infinity. Make the restitution amount configurable; you should see things settle down faster with less restitution, but the final "settled" state should take up the same amount of space no matter the amount of restitution.


Most of the project is finished, you just need to clean things up.

Priority number one:

Right now some balls escape the container by phasing through the container walls, this needs to be fixed.

Priority number two:

The balls never fully settle down, they just continue to bounce against eachother forever. this needs to be fixed.

In order to solve this efficiently you should create a well put together test suite that you can use to verify results.
Do not cut corners or hard code tests to pass.

Please work efficiently, if there is a large task, split it up into chunks and divide among

When both priorities have been solved, please terminate your process (you are being run in a loop)
