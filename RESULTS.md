# Week 10
I ran into quite a few issues just getting the environment set up. SDL3 wouldn’t install or run correctly at first, and it took a fair amount of troubleshooting to get everything working properly.

I also spent a lot more time than I should have trying to get the AMP loop command running, only to realize the issue was that we were out of credits.

Once SDL3 was configured, my loop actually worked fairly well. The main problems were significant lag and occasional cases where balls would phase through walls. The lag was resolved pretty quickly, but the collision issue took longer. I would get it to a point where everything behaved correctly for around 10 seconds, then a ball would randomly escape again.

To improve performance, I adjusted my initial prompt to have Codex optimize for timing by explicitly targeting 60 FPS. After that, I reinforced the constraint multiple times in the prompt to prevent balls from phasing through walls, which eventually fixed the issue.

Overall, I’m not sure I like this approach that much yet. That’s probably partly because it’s new and I’m still getting used to it, but I find myself preferring a more hands-on, iterative prompting style. This method felt slower than expected, and I often felt like I could have completed the same work more efficiently with a more direct approach. It also seems to get stuck at times, and I worry that it may be making unnecessary changes simply because it’s running in an infinite loop.

# Week 11