#!/usr/bin/env bash
# One-shot: turn this folder into a git repo and push it to a NEW PRIVATE GitHub repo.
#   cd ~/Documents/blimp-bot-interaction && bash push_to_github.sh [repo-name]
# Needs: git, and the GitHub CLI logged in (gh auth login). Without gh it prints the manual steps.
set -euo pipefail
cd "$(dirname "$0")"
NAME="${1:-blimp-bot-interaction}"

if [ ! -d .git ]; then
  git init -b main
fi
git add -A
if git diff --cached --quiet; then
  echo "nothing new to commit"
else
  git commit -m "Blimp + rover team simulator v1: 6-DoF GT-MAB blimp, unicycle rovers, PID on CM, 3D viewer with HUD/sliders/PID button, BlueROV-style teleop (Q up / E down), verification scripts"
fi

if command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
  if gh repo view "$NAME" >/dev/null 2>&1; then
    echo "repo $NAME already exists on GitHub; pushing to it"
    git remote get-url origin >/dev/null 2>&1 || git remote add origin "$(gh repo view "$NAME" --json url -q .url).git"
    git push -u origin main
  else
    gh repo create "$NAME" --private --source=. --remote=origin --push
  fi
  gh repo view "$NAME" --json url -q .url
else
  cat <<MSG
GitHub CLI not available or not logged in. Either:
  gh auth login            # then re-run this script
or create an empty PRIVATE repo named $NAME at https://github.com/new and run:
  git remote add origin git@github.com:<your-user>/$NAME.git
  git push -u origin main
MSG
fi
