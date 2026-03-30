The project description is as follows:
Implement a 2D physics simulator. The simulator should simulate circular balls and fixed, immovable walls. The balls should be influenced by gravity and should collide with the walls and with other balls in non-elastic collisions with restitution. If you don't know what that means, that's OK, Amp probably does.

Use SDL3 via C++. Running your application should set up an initial "scene" with about a thousand balls in some kind of container made up of wall pieces. Set up the initial scene so the balls bounce around for a while and then settle down. Ideally it'll also be fast enough that it's pleasant to look at.

The hard part is going to be making sure the balls don't end up overlapping or squeezing through the walls. You'll also sometimes see balls start vibrating really fast. Sometimes they vibrate fast and faster and eventually shoot off to infinity. Make the restitution amount configurable; you should see things settle down faster with less restitution, but the final "settled" state should take up the same amount of space no matter the amount of restitution.

Priority number one:

Make it possible to describe the initial scene in a CSV file; the CSV file should have one row per ball and list a starting position and a color. (The walls can be fixed, or you can add them and even other fields, like size, if you want.) Make the simulator save the final positions to a similar CSV file. Add a tool that takes an initial scene CSV file and assign colors based on where the final balls end up and what color a given image has at that location.

If you do it right, you should be able to run the simulator, have a lot of colorful balls bounce around seemingly at random, and then when things settle down you should be able to see an image form. There are videos on Youtube where you can see this in action.

Priority number two:

Right now some balls escape the container by phasing through the container walls, this needs to be fixed.

The balls never fully settle down, they just continue to bounce against eachother forever. this needs to be fixed.

In order to solve this efficiently you should create a well put together test suite that you can use to verify results.
Do not cut corners or hard code tests to pass.

Please work efficiently, if there is a large task, split it up into chunks and divide among

When both priorities have been solved proceed to this task,

For both the physics logic and the csv logic, you should use the folder "reference" for reference logic.

