# Collaboration

Use [README](README.md), [run guide](docs/RUNNING.md), and the [implementation audit](docs/IMPLEMENTATION_AUDIT.md) as the current entry points. Previous guides are preserved under `archive/original-guides/`.

1. Check `git status` before changing branches or updating. Preserve untracked/ignored files separately; Git does not protect them automatically.
2. Fetch remote changes and review divergence before merging or rebasing. Do not overwrite another contributor's work or force-push shared history.
3. Use a focused branch and review the diff. Keep documentation cleanup separate from behavior changes.
4. Run relevant offline checks. State explicitly whether a change was tested with a synthetic input, browser simulator, actual Myo, or physical robot.
5. Push and open a PR only after the repository owner approves publication. This cleanup branch has not been pushed.

Keep controller versions, recorded datasets, source archives, and historical logs unless their removal is explicitly agreed. Review new recordings/logs before committing. Keep environments, credentials and caches out of Git.

The Myo publisher, bridge and simulator base environment were provided separately; avoid attributing those components or all imported files to any single team member. Git authorship is evidence of a commit, not proof of sole implementation. Attribution and resume outcomes will be discussed separately.
