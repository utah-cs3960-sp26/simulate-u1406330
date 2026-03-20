Requirements
Implement a 2D physics simulator. The simulator should simulate circular balls and fixed, immovable walls. The balls should be influenced by gravity and should collide with the walls and with other balls in non-elastic collisions with restitution. If you don't know what that means, that's OK, Amp probably does.

Use SDL3 via C++. Running your application should set up an initial "scene" with about a thousand balls in some kind of container made up of wall pieces. Set up the initial scene so the balls bounce around for a while and then settle down. Ideally it'll also be fast enough that it's pleasant to look at.

The hard part is going to be making sure the balls don't end up overlapping or squeezing through the walls. You'll also sometimes see balls start vibrating really fast. Sometimes they vibrate fast and faster and eventually shoot off to infinity. Make the restitution amount configurable; you should see things settle down faster with less restitution, but the final "settled" state should take up the same amount of space no matter the amount of restitution.

This is very important - make sure none of the balls phase through the walls or go through tiny holes, they should all stay in the container.

Please write clean code and think about the repercussions of every implementation

for example

"Will this be supportable long term?"

"Is this extremely hard to test?"

"Is this the most efficient and clean way to do this?"

After every implementation, go back and clean up dead code, update readme with up to date project information, If you think whatever you implemented should be tested please test it.

Only write smart tests that are actually useful.

Whenever applicable, break implementations down into simpler parts, and divide the work amongst subagents.

Make the problem feedback-loopable for yourself as you work. Do not rely only on the interactive SDL window to judge correctness.

Build a small playground or debugging harness around the simulation so that the behavior is easy to inspect and easy to validate.

Set up reproducible experiments for tricky cases. If you find or suspect a bug, create a deterministic way to replay that exact scenario again instead of relying on manual timing or luck. Seed randomness where needed, and make initial conditions configurable.

Create a fast inner loop for debugging. Whenever possible, add a headless or CLI mode that can run the simulation without the full UI and print useful text output for validation, such as frame-by-frame positions, velocities, collision events, overlap counts, penetration depth, or energy trends.

Use the fast textual/debugging loop for iteration, and use the SDL visualization for final end-to-end validation.

When a bug is hard to see in a live animation, add tooling that turns it into something easier to inspect, such as a paused snapshot, step-by-step replay, saved scenario, or static output.

Design your debugging tools so they are low-friction to run repeatedly. It should be easy to vary parameters like restitution, timestep, ball count, spawn layout, seed, and scenario shape.

Do not just patch visible symptoms. Use the reproducible experiments and debug tooling to validate the actual underlying fix, and check nearby edge cases too.

Make sure the program runs at minimum 60fps, and no balls phase through walls unnecesarily.