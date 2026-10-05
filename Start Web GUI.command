#!/bin/zsh
cd -- "$(dirname "$0")"
if [[ ! -x .venv/bin/python ]]; then
  print "Set up the local environment once, then launch this file again:"
  print "python3 -m venv .venv"
  print ".venv/bin/python -m pip install -r requirements-web.txt"
  read "?Press Enter to close."
  exit 1
fi
exec .venv/bin/python web_server.py "$@"
