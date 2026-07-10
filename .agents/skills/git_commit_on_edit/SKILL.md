---
name: git-commit-on-edit
description: Automatically commit modifications or creations of files to Git with a short 3-4 word English message. Use this skill whenever any file is edited, created, or deleted.
---
# Git Commit on Edit Skill

This skill enforces committing every file modification, creation, or deletion to Git immediately after the file edit/creation tool completes.

## Instructions:
1. Stage the modified, created, or deleted file(s) using git add:
   `git add <path_to_file>`
2. Commit the changes using git commit:
   `git commit -m "<message>"`
3. The commit message MUST be extremely short, consisting of exactly 3 to 4 English words (e.g., "update joint kinematics", "fix encoder ticks", "sync left firmware", "create test script").
4. This commit must be executed immediately after the edit/creation and before doing other tasks.
