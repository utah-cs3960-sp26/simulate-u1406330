# Week 10

I ran into quite a few issues just getting the environment set up. SDL3 wouldn’t install or run correctly at first, and it took a fair amount of troubleshooting to get everything working properly.

I also spent a lot more time than I should have trying to get the AMP loop command running, only to realize the issue was that we were out of credits.

Once SDL3 was configured, my loop actually worked fairly well. The main problems were significant lag and occasional cases where balls would phase through walls. The lag was resolved pretty quickly, but the collision issue took longer. I would get it to a point where everything behaved correctly for around 10 seconds, then a ball would randomly escape again.

To improve performance, I adjusted my initial prompt to have Codex optimize for timing by explicitly targeting 60 FPS. After that, I reinforced the constraint multiple times in the prompt to prevent balls from phasing through walls, which eventually fixed the issue.

Overall, I’m not sure I like this approach that much yet. That’s probably partly because it’s new and I’m still getting used to it, but I find myself preferring a more hands-on, iterative prompting style. This method felt slower than expected, and I often felt like I could have completed the same work more efficiently with a more direct approach. It also seems to get stuck at times, and I worry that it may be making unnecessary changes simply because it’s running in an infinite loop.

# Week 11

For this part of the assignment, I had to redo my physics engine because the balls would never fully settle. To speed up the process, I took inspiration from JustHTML and cloned the repository from the YouTube video referenced in the assignment. I then pointed Codex to that repository so it could use it for reference if it needed ideas for the physics implementation. After that, I provided Codex with the Week 11 assignment description along with some of the main issues we were having with our physics simulator and let it run.

I also changed my prompt strategy a bit. At the beginning I had a lot of “tips” written for the agent, things like “write useful tests” and “make your test suite feedback-loopable.” Some of those ideas probably helped early on, but eventually the prompt started to feel cluttered.

To fix that, I simplified it quite a bit. I removed most of the extra advice and kept only the core pieces: a short program description to give the agent context, the main priorities (implement the CSV logic and fix the physics), and a few helpful tips. This ended up working better than the more cluttered approach where I just kept adding anything that seemed useful into the prompt.

I ran out of credits once and had to restart it the next morning. The full process took around four hours, but it eventually reached its current state where it can convert an image into a CSV file and map the colors to the resting positions of the balls.

The most annoying behavior I saw from Codex was that, in several attempts to make the balls settle, it simply disabled bouncing entirely. It did this multiple times. To prevent that from happening again, I added a rule in agents.md that says, “never force-settle the pile by killing bounce.” This helped stop Codex from using that shortcut.
