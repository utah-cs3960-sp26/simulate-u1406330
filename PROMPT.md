Requirements
Implement a 2D physics simulator. The simulator should simulate circular balls and fixed, immovable walls. The balls should be influenced by gravity and should collide with the walls and with other balls in non-elastic collisions with restitution. If you don't know what that means, that's OK, Amp probably does.

Use SDL3 via C++. Running your application should set up an initial "scene" with about a thousand balls in some kind of container made up of wall pieces. Set up the initial scene so the balls bounce around for a while and then settle down. Ideally it'll also be fast enough that it's pleasant to look at.

The hard part is going to be making sure the balls don't end up overlapping or squeezing through the walls. You'll also sometimes see balls start vibrating really fast. Sometimes they vibrate fast and faster and eventually shoot off to infinity. Make the restitution amount configurable; you should see things settle down faster with less restitution, but the final "settled" state should take up the same amount of space no matter the amount of restitution.

This is very important - make sure none of the balls phase through the walls or go through tiny holes, they should all stay in the container.

Please write clean code and think about the reprecusions of every implementation

for example

"Will this be supportable long term?"

"Is this extremely hard to test?"

"Is this the most efficient and clean way to do this?"

After every implementation, go back and clean up dead code, update readme with up to date project information, If you think whatever you implemented should be tested please test it.

Only write smart tests that are actually useful.


